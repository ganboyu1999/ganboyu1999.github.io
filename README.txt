Zhuocheng Han — Personal Homepage & Selected Projects

A minimal personal homepage for robotics and autonomous driving.
Static HTML and CSS; no build step, JavaScript, or runtime dependencies.

Files:
index.html — profile, selected projects, work experience, education, contact.
style.css — responsive styles with automatic light/dark mode.
favicon.svg — ZH icon.
.nojekyll — serve the static files directly on GitHub Pages.

Preview: python3 -m http.server 8773
GitHub Pages: deploy the main branch, root directory (/).

Portfolio maintenance:
Each <article class="project"> contains a title, short description, diagram,
results, and a <details> case study with problem, contributions, and outcomes.
The diagrams are explanatory illustrations based on the owner's supplied facts.
To add real screenshots or demo videos, place approved media in assets/ and
replace the corresponding <figure class="project-preview"> with an image or
<video controls preload="metadata" playsinline> element. Use relative paths,
add a caption and accessible text, and retain the project-specific details.
Add public source or documentation links only when a real public URL exists.

Layout reference: https://ddod-jim.github.io/
The homepage uses the owner’s chosen display name and omits the portrait, phone, and original resume.
