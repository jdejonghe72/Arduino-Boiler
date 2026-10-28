#!/usr/bin/env python3
"""Lire / modifier un projet Nextion Editor (.HMI) sans Nextion Editor.

  python3 hmi_tool.py dump   "Interface Chaudiere Atom.HMI"           # code + attributs, lisible
  python3 hmi_tool.py export "Interface Chaudiere Atom.HMI" pages/    # 1 fichier JSON par page
  python3 hmi_tool.py import "Interface Chaudiere Atom.HMI" pages/    # reinjecte les JSON modifies

Limites (format proprietaire) :
  - une page est reecrite A TAILLE IDENTIQUE : si elle raccourcit, un commentaire "//---" est
    ajoute dans l'evenement "Page Exit" de la page ; si elle grossit, l'import est refuse
    (supprimer des commentaires ailleurs dans la page) ;
  - pas d'ajout/suppression de composant, de page, d'image ou de police (Nextion Editor) ;
  - toujours rouvrir le fichier dans Nextion Editor avant de compiler.

Format : conteneur = repertoire de blocs (nom, offset, taille) ; pages "N.pa" signees par
stamp = CRC32(octets[4:] elargis en mots 32 bits + 40 zeros) ^ K(taille).
K ne depend que de la taille : il est recalcule a partir du bloc d'origine.
Reference : https://github.com/vladtsvetkov880-hub/nextion-hmi-format
"""
import json
import os
import struct
import sys

ENC = 'latin-1'  # aller-retour sans perte ; les polices du projet sont en iso-8859-1
PAGE_HDR = 0x38
PAD_EVENT = 'codesunload'

# ----------------------------------------------------------------------------
# Stamp des pages (CRC32 MSB-first, poly 0x04C11DB7, init 0, sans XOR final)
# ----------------------------------------------------------------------------
_T = []
for _i in range(256):
    _c = _i << 24
    for _ in range(8):
        _c = ((_c << 1) ^ 0x04C11DB7) & 0xFFFFFFFF if _c & 0x80000000 else (_c << 1) & 0xFFFFFFFF
    _T.append(_c)


def _crc_widened(block):
    crc = 0
    t = _T
    for b in block[4:]:
        # chaque octet = mot big-endian 00 00 00 b
        crc = ((crc << 8) & 0xFFFFFFFF) ^ t[crc >> 24]
        crc = ((crc << 8) & 0xFFFFFFFF) ^ t[crc >> 24]
        crc = ((crc << 8) & 0xFFFFFFFF) ^ t[crc >> 24]
        crc = ((crc << 8) & 0xFFFFFFFF) ^ t[(crc >> 24) ^ b]
    for _ in range(40):
        crc = ((crc << 8) & 0xFFFFFFFF) ^ t[crc >> 24]
    return crc


def stamp_constant(block):
    """K(taille) d'un bloc signe par Nextion Editor."""
    return struct.unpack_from('<I', block, 0)[0] ^ _crc_widened(block)


def restamp(block, k):
    return struct.pack('<I', _crc_widened(block) ^ k) + block[4:]


# ----------------------------------------------------------------------------
# Conteneur
# ----------------------------------------------------------------------------
class Container:
    def __init__(self, path):
        self.path = path
        self.data = bytearray(open(path, 'rb').read())
        n = struct.unpack_from('<I', self.data, 0)[0]
        self.entries = []
        for i in range(n):
            off = 4 + 28 * i
            name = bytes(self.data[off:off + 16]).split(b'\0')[0].decode(ENC)
            start, size, flags = struct.unpack_from('<III', self.data, off + 16)
            if not flags & 0xFF:
                self.entries.append((name, start, size))

    def get(self, name):
        for n, start, size in self.entries:
            if n == name:
                return bytes(self.data[start:start + size])
        raise KeyError(name)

    def put_same_size(self, name, block):
        for n, start, size in self.entries:
            if n == name:
                if len(block) != size:
                    raise ValueError('%s : taille %d au lieu de %d' % (name, len(block), size))
                self.data[start:start + size] = block
                return
        raise KeyError(name)

    def page_files(self):
        """Fichiers .pa dans l'ordre des pages (liste de ressources de main.HMI)."""
        m = self.get('main.HMI')
        hsize = struct.unpack_from('<I', m, 4)[0]
        files = []
        for off in range(hsize, len(m) - 15, 16):
            kind = m[off:off + 8].split(b'\0')[0]
            name = m[off + 8:off + 16].split(b'\0')[0].decode(ENC)
            if kind == b'pa':
                files.append(name)
        return files

    def save(self, path=None):
        open(path or self.path, 'wb').write(self.data)


# ----------------------------------------------------------------------------
# Pages : composant = suite d'enregistrements [u32 longueur][octets]
#   "att-N", N attributs (nom sur 16 octets + valeur), puis evenements
#   "codesXXX-K" suivis de K lignes de code, puis un enregistrement vide.
# ----------------------------------------------------------------------------
def _records(raw):
    recs, p = [], 0
    while p < len(raw):
        (n,) = struct.unpack_from('<I', raw, p)
        recs.append(raw[p + 4:p + 4 + n])
        p += 4 + n
    return recs


def parse_page(block):
    ncomp = struct.unpack_from('<I', block, 12)[0]
    comps = []
    for i in range(ncomp):
        off, size, _ = struct.unpack_from('<III', block, PAGE_HDR + 12 * i)
        recs = _records(block[PAGE_HDR + off:PAGE_HDR + off + size])
        natt = int(recs[0].decode(ENC).split('-')[1])
        atts = [(r[:16].split(b'\0')[0].decode(ENC), r[16:]) for r in recs[1:1 + natt]]
        events, i_rec = [], 1 + natt
        while i_rec < len(recs):
            r = recs[i_rec]
            if not r:
                i_rec += 1
                continue
            ev, k = r.decode(ENC).rsplit('-', 1)
            k = int(k)
            events.append([ev, [x.decode(ENC) for x in recs[i_rec + 1:i_rec + 1 + k]]])
            i_rec += 1 + k
        comps.append({'atts': atts, 'events': events})
    return bytes(block[:PAGE_HDR]), comps


def _rec(b):
    return struct.pack('<I', len(b)) + b


def build_page(header, comps):
    bodies = []
    for c in comps:
        out = [_rec(('att-%d' % len(c['atts'])).encode())]
        for name, val in c['atts']:
            out.append(_rec(name.encode(ENC).ljust(16, b'\0') + val))
        for ev, lines in c['events']:
            out.append(_rec(('%s-%d' % (ev, len(lines))).encode(ENC)))
            out += [_rec(line.encode(ENC)) for line in lines]
        out.append(_rec(b''))
        bodies.append(b''.join(out))
    table, off = [], 12 * len(comps)
    for body in bodies:
        table.append(struct.pack('<III', off, len(body), 0))
        off += len(body)
    size = PAGE_HDR + off
    hdr = bytearray(header)
    struct.pack_into('<I', hdr, 4, size)
    struct.pack_into('<I', hdr, 12, len(comps))
    return bytes(hdr) + b''.join(table) + b''.join(bodies)


def fit_to_size(header, comps, target):
    """Construit la page a la taille exacte `target` (bourrage dans PAD_EVENT de la page)."""
    pad_ev = next((ev for ev in comps[0]['events'] if ev[0] == PAD_EVENT), None)
    if pad_ev is not None:  # retire le bourrage d'un import precedent
        pad_ev[1][:] = [l for l in pad_ev[1] if not l.startswith('//---')]
    missing = target - len(build_page(header, comps))
    if missing < 0:
        raise ValueError('page trop grande de %d octets : supprime des commentaires' % -missing)
    if missing == 0:
        return build_page(header, comps)
    if pad_ev is not None:
        pad_ev[1].append('//---')
        extra = target - len(build_page(header, comps))
        if extra >= 0:
            pad_ev[1][-1] += '-' * extra
            return build_page(header, comps)
        pad_ev[1].pop()
    # moins de 9 octets a combler : on rallonge un commentaire existant
    for c in comps:
        for _, lines in c['events']:
            for j, l in enumerate(lines):
                if l.lstrip().startswith('//'):
                    lines[j] = l + '-' * missing
                    return build_page(header, comps)
    raise ValueError('impossible de combler %d octets (ajoute un commentaire)' % missing)


# ----------------------------------------------------------------------------
# Export / import JSON (entiers : [nom, largeur, valeur] ; textes : [nom, "s", texte])
# ----------------------------------------------------------------------------
TEXT_ATTS = {'objname', 'txt', 'path', 'dir', 'filter', 'tim_txt', 'format'}


def comps_to_json(comps):
    out = []
    for c in comps:
        atts = []
        for name, val in c['atts']:
            if name in TEXT_ATTS or len(val) not in (1, 2, 4):
                atts.append([name, 's', val.decode(ENC)])
            else:
                atts.append([name, len(val), int.from_bytes(val, 'little')])
        out.append({'atts': atts, 'events': c['events']})
    return out


def comps_from_json(jcomps):
    comps = []
    for c in jcomps:
        atts = []
        for name, width, val in c['atts']:
            atts.append((name, val.encode(ENC) if width == 's' else int(val).to_bytes(width, 'little')))
        comps.append({'atts': atts, 'events': [[ev, list(lines)] for ev, lines in c['events']]})
    return comps


def _val(name, v):
    if name in TEXT_ATTS or len(v) not in (1, 2, 4):
        return repr(v.decode(ENC))
    return str(int.from_bytes(v, 'little'))


def dump(cont):
    out = []
    for idx, f in enumerate(cont.page_files()):
        _, comps = parse_page(cont.get(f))
        out.append('#' * 72)
        out.append('# page %d (%s)' % (idx, f))
        for c in comps:
            a = dict(c['atts'])
            out.append('--- %s  type=%d id=%d' % (a['objname'].decode(ENC), a['type'][0], a['id'][0]))
            out.append('    ' + ' '.join('%s=%s' % (k, _val(k, v)) for k, v in c['atts']
                                         if k not in ('objname', 'type', 'id')))
            for ev, lines in c['events']:
                if lines:
                    out.append('    [%s]' % ev)
                    out += ['      | ' + l for l in lines]
    return '\n'.join(out) + '\n'


def main(argv):
    if len(argv) < 3 or argv[1] not in ('dump', 'export', 'import'):
        print(__doc__)
        return 1
    cont = Container(argv[2])
    if argv[1] == 'dump':
        sys.stdout.write(dump(cont))
        return 0
    outdir = argv[3]
    if argv[1] == 'export':
        os.makedirs(outdir, exist_ok=True)
        for f in cont.page_files():
            header, comps = parse_page(cont.get(f))
            if build_page(header, comps) != cont.get(f):
                raise SystemExit('%s : relecture non identique, format non supporte' % f)
            name = dict(comps[0]['atts'])['objname'].decode(ENC)
            with open(os.path.join(outdir, name + '.json'), 'w', encoding='utf-8') as fp:
                json.dump({'file': f, 'components': comps_to_json(comps)}, fp, ensure_ascii=False, indent=1)
        print('pages exportees dans', outdir)
        return 0
    changed = 0
    for jf in sorted(os.listdir(outdir)):
        if not jf.endswith('.json'):
            continue
        with open(os.path.join(outdir, jf), encoding='utf-8') as fp:
            j = json.load(fp)
        old = cont.get(j['file'])
        header, _ = parse_page(old)
        comps = comps_from_json(j['components'])
        new = fit_to_size(header, comps, len(old))
        new = restamp(new, stamp_constant(old))
        if new != old:
            cont.put_same_size(j['file'], new)
            changed += 1
            print('%s : %s reecrit (%d octets)' % (jf, j['file'], len(new)))
    cont.save()
    print('%d page(s) modifiee(s). Ouvre le fichier dans Nextion Editor pour verifier.' % changed)
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv))
