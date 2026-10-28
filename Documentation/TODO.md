# TODO:

- [x] Ajouter Bouton pour forcer Idle/CoolDown
- [x] Ajouter Indicateurs de fonctionnement (fire)
- [x] Ajouter programmation SETDAY sur l'écran Nextion
  - [x] Ajouter animation de loading de la configuration
  - [x] re-LOAD après envoi de la config.
  - [ ] couleurs & icônes (-->)
  - [x] Changer le texte du bouton autoload
- [x] STOP/START possibles quand en programmation et START possible quand CoolDown
- [x] Ajouter une vue de la température/indication de défaut de pellets/etc. sur l'écran Nextion (bloc d'état page 0)
- [x] Force CoolDown doit durer 10 min
- [ ] Ouvrir le .HMI dans Nextion Editor, compiler et charger le TFT sur l'écran
- [ ] Vérifier en RUN que le pressostat dépression ne déclenche pas à tort (sinon `ENABLE_NO_DEP_ALARM 0`)
