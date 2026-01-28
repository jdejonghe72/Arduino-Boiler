# Test Theme Syntax

## Using picture with #gh-dark-mode-only and #gh-light-mode-only

<picture>
  <source media="(prefers-color-scheme: dark)" srcset="#gh-dark-mode-only">
  <source media="(prefers-color-scheme: light)" srcset="#gh-light-mode-only">
</picture>

<div id="gh-dark-mode-only">
Dark mode content (white text)
</div>

<div id="gh-light-mode-only">
Light mode content (black text)
</div>

## Direct approach with divs and IDs

<div id="user-content-dark-mode" style="display:none;">
**Dark Mode**: White text goes here
</div>

<div id="user-content-light-mode">
**Light Mode**: Black text goes here
</div>

