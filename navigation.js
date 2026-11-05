const directory = document.querySelector('.mobile-directory');
const directoryLinks = document.querySelectorAll('.side-nav a, .mobile-directory nav a');
const sections = ['experience', 'works'].map(id => document.getElementById(id));
let scrollPending = false;

function updateCurrentSection() {
  let current = null;
  for (const section of sections) {
    if (section.getBoundingClientRect().top <= 140) current = section.id;
  }
  for (const link of directoryLinks) {
    if (link.hash === `#${current}`) link.setAttribute('aria-current', 'location');
    else link.removeAttribute('aria-current');
  }
  scrollPending = false;
}

for (const link of directoryLinks) {
  link.addEventListener('click', () => { directory.open = false; });
}
document.addEventListener('click', event => {
  if (!directory.contains(event.target)) directory.open = false;
});
document.addEventListener('keydown', event => {
  if (event.key === 'Escape' && directory.open) {
    directory.open = false;
    directory.querySelector('summary').focus();
  }
});
window.addEventListener('scroll', () => {
  if (!scrollPending) {
    scrollPending = true;
    requestAnimationFrame(updateCurrentSection);
  }
}, { passive: true });
window.addEventListener('resize', updateCurrentSection);
window.addEventListener('pageshow', updateCurrentSection);
document.addEventListener('toggle', updateCurrentSection, true);
updateCurrentSection();
