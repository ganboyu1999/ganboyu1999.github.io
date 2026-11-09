"use strict";

const pages = window.FRS_PAGES;
const navigation = document.querySelector("#navigation");
const sectionNavigation = document.querySelector("#section-navigation");
const lesson = document.querySelector("#lesson");
const search = document.querySelector("#search");
const results = document.querySelector("#search-results");
const toggle = document.querySelector("#menu-toggle");
const languageToggle = document.querySelector("#language-toggle");
const languageMenu = document.querySelector("#language-menu");
let sectionObserver;
let toastTimer;
let codeLanguage = "python";

function selectCodeLanguage(language) {
  for (const group of lesson.querySelectorAll(".api-examples")) {
    const tabs = [...group.querySelectorAll('[role="tab"]')];
    const selected = tabs.find(tab => tab.dataset.codeLanguage === language) || tabs[0];
    if (!selected) continue;
    for (const tab of tabs) {
      const active = tab === selected;
      tab.setAttribute("aria-selected", String(active));
      tab.tabIndex = active ? 0 : -1;
      group.querySelector(`#${tab.getAttribute("aria-controls")}`).hidden = !active;
    }
  }
}

function escapeText(value) {
  const span = document.createElement("span");
  span.textContent = value;
  return span.innerHTML;
}

function appendPageLinks(container, entries) {
  for (const page of entries) {
    const anchor = document.createElement("a");
    anchor.href = `#${page.id}`;
    anchor.dataset.page = page.id;
    anchor.textContent = page.title;
    container.append(anchor);
  }
}

for (const group of [...new Set(pages.map(page => page.group))]) {
  const groupPages = pages.filter(page => page.group === group);
  const category = document.createElement("a");
  category.href = `#${groupPages[0].id}`;
  category.dataset.group = group;
  category.textContent = group;
  sectionNavigation.append(category);
  const details = document.createElement("details");
  details.dataset.group = group;
  const summary = document.createElement("summary");
  summary.append(document.createTextNode(group));
  const links = document.createElement("div");
  links.className = "group-links";
  for (const category of [...new Set(groupPages.map(page => page.category || ""))]) {
    const entries = groupPages.filter(page => (page.category || "") === category);
    if (!category || (entries.length === 1 && entries[0].title === category)) {
      appendPageLinks(links, entries);
      continue;
    }
    const subgroup = document.createElement("details");
    subgroup.className = "nav-category";
    subgroup.dataset.category = category;
    const heading = document.createElement("summary");
    heading.textContent = category;
    const children = document.createElement("div");
    children.className = "category-links";
    appendPageLinks(children, entries);
    subgroup.append(heading, children);
    links.append(subgroup);
  }
  details.append(summary, links);
  navigation.append(details);
}

function hideSearch() {
  results.hidden = true;
  search.setAttribute("aria-expanded", "false");
}

function hideLanguage() {
  languageMenu.hidden = true;
  languageToggle.setAttribute("aria-expanded", "false");
}

function showToast(message) {
  const toast = document.querySelector("#toast");
  toast.textContent = message;
  toast.classList.add("visible");
  clearTimeout(toastTimer);
  toastTimer = setTimeout(() => toast.classList.remove("visible"), 2200);
}

function highlightCode() {
  for (const block of lesson.querySelectorAll("pre code")) {
    if (!/language-(python|cpp|json|cmake|bash|sh)/.test(block.className)) continue;
    const text = block.textContent;
    const pattern = /("(?:\\.|[^"\\])*"|'(?:\\.|[^'\\])*'|\/\/[^\n]*|#[^\n]*|\b(?:import|from|def|class|return|raise|if|else|elif|for|in|with|as|try|except|finally|const|auto|int|void|using|namespace|include|true|false|null|True|False|None)\b|\b\d+(?:\.\d+)?\b)/g;
    let cursor = 0;
    let formatted = "";
    for (const match of text.matchAll(pattern)) {
      formatted += escapeText(text.slice(cursor, match.index));
      const token = match[0];
      const kind = /^['"]/.test(token) ? "string" : /^(#|\/\/)/.test(token) ? "comment" : /^\d/.test(token) ? "number" : "keyword";
      formatted += `<span class="token-${kind}">${escapeText(token)}</span>`;
      cursor = match.index + token.length;
    }
    block.innerHTML = formatted + escapeText(text.slice(cursor));
  }
}

function render() {
  const redirect = pages.map(page => (page.redirects || {})[location.hash.slice(1)]).find(Boolean);
  if (redirect) history.replaceState(null, "", `#${redirect}`);
  const [requested, sectionId] = location.hash.slice(1).split("/");
  const page = pages.find(page => page.id === requested || (page.aliases || []).includes(requested)) || pages[0];
  if ((page.aliases || []).includes(requested)) history.replaceState(null, "", `#${page.id}${sectionId ? "/" + sectionId : ""}`);
  const index = pages.indexOf(page);
  document.title = `${page.title} · FRS 开发者文档`;
  const breadcrumb = [page.group, page.category === page.title ? "" : page.category, page.title].filter(Boolean)
    .map(value => `<span>›</span>${escapeText(value)}`).join("");
  lesson.innerHTML = `<nav class="breadcrumb" aria-label="当前位置"><a href="#overview">FRS</a>${breadcrumb}</nav>` +
    `<header class="document-header"><h1>${escapeText(page.title)}</h1><button id="edit-document" type="button" hidden>编辑</button></header>` +
    page.sections.map(section => `<section id="${section.id}"><h2><span class="section-title">${escapeText(section.title)}</span><a class="heading-anchor" href="#${page.id}/${section.id}" aria-label="链接到 ${escapeText(section.title)}">#</a></h2><div class="section-body">${section.html}</div></section>`).join("");
  for (const anchor of navigation.querySelectorAll("a")) {
    const linked = pages.find(item => item.id === anchor.dataset.page);
    if (linked) anchor.textContent = linked.title;
    if (anchor.dataset.page === page.id) anchor.setAttribute("aria-current", "page");
    else anchor.removeAttribute("aria-current");
  }
  for (const group of navigation.querySelectorAll("details[data-group]")) {
    group.open = group.dataset.group === page.group;
  }
  for (const category of navigation.querySelectorAll("details[data-category]")) {
    category.open = category.closest("details[data-group]").dataset.group === page.group &&
      category.dataset.category === page.category;
  }
  for (const category of sectionNavigation.querySelectorAll("a")) {
    if (category.dataset.group === page.group) category.setAttribute("aria-current", "true");
    else category.removeAttribute("aria-current");
  }
  const toc = document.querySelector("#toc-links");
  toc.innerHTML = page.sections.map(section => `<a href="#${page.id}/${section.id}" data-section="${section.id}">${escapeText(section.title)}</a>`).join("");
  const pageLink = (other, label, next) => other ? `<a class="${next ? "next" : "previous"}" href="#${other.id}"><small>${label}</small><strong>${next ? "" : "← "}${escapeText(other.title)}${next ? " →" : ""}</strong></a>` : "<span></span>";
  document.querySelector("#pagination").innerHTML = pageLink(pages[index - 1], "上一篇", false) + pageLink(pages[index + 1], "下一篇", true);
  highlightCode();
  selectCodeLanguage(codeLanguage);
  if (sectionObserver) sectionObserver.disconnect();
  sectionObserver = new IntersectionObserver(entries => {
    const visible = entries.find(entry => entry.isIntersecting);
    if (visible) for (const anchor of toc.querySelectorAll("a")) {
      anchor.classList.toggle("active", anchor.dataset.section === visible.target.id);
    }
  }, {rootMargin: "-120px 0px -60% 0px"});
  lesson.querySelectorAll("section").forEach(section => sectionObserver.observe(section));
  document.body.classList.remove("menu-open");
  toggle.setAttribute("aria-expanded", "false");
  hideSearch();
  if (sectionId && document.getElementById(sectionId)) {
    const target = document.getElementById(sectionId);
    for (const detail of target.querySelectorAll(":scope > .section-body > details")) detail.open = true;
    target.scrollIntoView();
  }
  else window.scrollTo(0, 0);
  window.dispatchEvent(new CustomEvent("frs:render", {detail: {page}}));
}

lesson.addEventListener("click", async event => {
  const tab = event.target.closest('[role="tab"][data-code-language]');
  if (tab) {
    codeLanguage = tab.dataset.codeLanguage;
    selectCodeLanguage(codeLanguage);
    return;
  }
  const button = event.target.closest(".copy-code");
  if (!button) return;
  const code = button.closest(".code-block").querySelector("code");
  try {
    await navigator.clipboard.writeText(code.textContent);
    button.textContent = "已复制";
    showToast("代码已复制");
    setTimeout(() => { button.textContent = "复制"; }, 1800);
  } catch {
    const selection = window.getSelection();
    const range = document.createRange();
    range.selectNodeContents(code);
    selection.removeAllRanges();
    selection.addRange(range);
    showToast("代码已选中，请按 Ctrl/Cmd+C 复制");
  }
});

lesson.addEventListener("keydown", event => {
  const tab = event.target.closest('[role="tab"][data-code-language]');
  if (!tab || !["ArrowLeft", "ArrowRight", "Home", "End"].includes(event.key)) return;
  event.preventDefault();
  const tabs = [...tab.closest('[role="tablist"]').querySelectorAll('[role="tab"]')];
  const index = tabs.indexOf(tab);
  const next = event.key === "Home" ? 0 : event.key === "End" ? tabs.length - 1 :
    (index + (event.key === "ArrowRight" ? 1 : -1) + tabs.length) % tabs.length;
  tabs[next].click();
  tabs[next].focus();
});

search.addEventListener("input", () => {
  const query = search.value.trim().toLocaleLowerCase();
  results.replaceChildren();
  if (!query) { hideSearch(); return; }
  const matches = pages.filter(page => `${page.title} ${page.group} ${page.category || ""} ${page.description} ${page.search}`.toLocaleLowerCase().includes(query));
  for (const page of matches) {
    const anchor = document.createElement("a");
    anchor.href = `#${page.id}`;
    const title = document.createElement("strong");
    title.textContent = page.title;
    const detail = document.createElement("span");
    detail.textContent = [page.group, page.category].filter(Boolean).join(" / ");
    anchor.append(title, detail);
    anchor.addEventListener("click", hideSearch);
    results.append(anchor);
  }
  if (!matches.length) results.textContent = "无搜索结果";
  results.hidden = false;
  search.setAttribute("aria-expanded", "true");
});

search.addEventListener("keydown", event => {
  if (event.key === "Enter" || event.key === "ArrowDown") {
    const first = results.querySelector("a");
    if (first) { event.preventDefault(); first.focus(); if (event.key === "Enter") first.click(); }
  }
});

document.addEventListener("keydown", event => {
  if ((event.metaKey || event.ctrlKey) && event.key.toLowerCase() === "k") {
    event.preventDefault();
    hideLanguage();
    search.focus();
  }
  if (event.key === "Escape") {
    if (!languageMenu.hidden) languageToggle.focus();
    hideLanguage();
    hideSearch(); document.body.classList.remove("menu-open");
    toggle.setAttribute("aria-expanded", "false");
  }
});
document.addEventListener("click", event => {
  if (!event.target.closest(".search-box")) hideSearch();
  if (!event.target.closest(".language-picker")) hideLanguage();
});
languageToggle.addEventListener("click", () => {
  languageMenu.hidden = !languageMenu.hidden;
  languageToggle.setAttribute("aria-expanded", String(!languageMenu.hidden));
});
languageMenu.querySelector("button").addEventListener("click", () => {
  hideLanguage();
  languageToggle.focus();
});
document.querySelector(".skip-link").addEventListener("click", event => {
  event.preventDefault();
  document.querySelector("#main").focus();
  window.scrollTo(0, 0);
});
toggle.addEventListener("click", () => {
  const open = document.body.classList.toggle("menu-open");
  toggle.setAttribute("aria-expanded", String(open));
});
window.addEventListener("hashchange", () => {
  if (window.FRS_EDITOR && !window.FRS_EDITOR.canNavigate()) return;
  render();
});
render();
