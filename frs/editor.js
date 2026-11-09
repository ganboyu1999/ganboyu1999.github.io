"use strict";

(() => {
  const toolbar = document.querySelector("#editor-toolbar");
  const state = document.querySelector("#editor-state");
  const saveButton = document.querySelector("#editor-save");
  const imageDialog = document.querySelector("#editor-image-dialog");
  const linkDialog = document.querySelector("#editor-link-dialog");
  let session, active, revision, savedRange, dirty = false, saving = false;

  function message(text, error = false) {
    state.textContent = text;
    state.classList.toggle("error", error);
  }

  function changed() {
    dirty = true;
    message("未保存");
  }

  function selectionRange() {
    const selection = window.getSelection();
    if (!selection.rangeCount) return null;
    const range = selection.getRangeAt(0);
    const node = range.commonAncestorContainer.nodeType === 1 ? range.commonAncestorContainer : range.commonAncestorContainer.parentElement;
    if (!node.closest('.section-body[contenteditable="true"]') || node.closest('[contenteditable="false"]')) return null;
    return range;
  }

  function rememberSelection() {
    const range = selectionRange();
    if (range) savedRange = range.cloneRange();
  }

  function restoreSelection() {
    if (!savedRange || !lesson.contains(savedRange.commonAncestorContainer)) {
      const body = lesson.querySelector('.section-body[contenteditable="true"]');
      savedRange = document.createRange();
      savedRange.selectNodeContents(body);
      savedRange.collapse(false);
    }
    const selection = window.getSelection();
    selection.removeAllRanges();
    selection.addRange(savedRange);
    return savedRange;
  }

  function insertHTML(html) {
    restoreSelection();
    document.execCommand("insertHTML", false, html);
    rememberSelection();
    changed();
  }

  function serializedBody(body) {
    const copy = body.cloneNode(true);
    for (const block of copy.querySelectorAll("[data-frs-block-id]")) {
      const placeholder = document.createElement("div");
      placeholder.dataset.frsBlock = block.dataset.frsBlockId;
      block.replaceWith(placeholder);
    }
    for (const token of copy.querySelectorAll('pre code span[class^="token-"]')) token.replaceWith(document.createTextNode(token.textContent));
    for (const element of copy.querySelectorAll("*")) {
      element.removeAttribute("contenteditable");
      element.removeAttribute("spellcheck");
      element.removeAttribute("role");
      element.removeAttribute("data-frs-block-id");
    }
    return copy.innerHTML;
  }

  async function request(path, options) {
    const response = await fetch(path, options);
    const value = await response.json();
    if (!response.ok) throw new Error(value.error || "保存失败");
    return value;
  }

  function editButton() {
    const button = lesson.querySelector("#edit-document");
    if (!button || !session) return;
    button.hidden = Boolean(active);
    button.onclick = begin;
  }

  async function begin() {
    const page = pages.find(page => page.id === location.hash.slice(1).split("/")[0]) || pages[0];
    const button = lesson.querySelector("#edit-document");
    button.disabled = true;
    try {
      const value = await request(`/__editor/pages/${page.id}`);
      active = value.page;
      revision = value.revision;
      Object.assign(page, active);
      render();
      document.body.classList.add("editing");
      toolbar.hidden = false;
      const heading = lesson.querySelector("h1");
      heading.contentEditable = "true";
      heading.setAttribute("aria-label", "文档标题");
      for (const section of lesson.querySelectorAll("section")) {
        const title = section.querySelector(".section-title");
        title.contentEditable = "true";
        title.setAttribute("aria-label", "小节标题");
        const body = section.querySelector(".section-body");
        body.contentEditable = "true";
        body.setAttribute("aria-label", `${title.textContent}正文`);
        for (const block of body.querySelectorAll("[data-frs-block-id]")) block.title = "示例与源码文件同步";
      }
      dirty = false;
      savedRange = null;
      message("编辑中");
      heading.focus();
    } catch (error) {
      showToast(error.message);
      button.disabled = false;
    }
  }

  function finish() {
    active = null;
    dirty = false;
    savedRange = null;
    toolbar.hidden = true;
    document.body.classList.remove("editing");
    render();
  }

  async function save() {
    if (!active || saving) return;
    const page = {
      id: active.id,
      title: lesson.querySelector("h1").textContent.trim(),
      description: active.description,
      sections: Array.from(lesson.querySelectorAll("section"), section => ({
        id: section.id,
        title: section.querySelector(".section-title").textContent.trim(),
        html: serializedBody(section.querySelector(".section-body"))
      }))
    };
    saving = true;
    toolbar.querySelectorAll("button").forEach(button => { button.disabled = true; });
    lesson.inert = true;
    message("正在保存…");
    try {
      const value = await request(`/__editor/pages/${active.id}`, {
        method: "POST", headers: {"Content-Type": "application/json", "X-FRS-Editor": session.token},
        body: JSON.stringify({revision, page})
      });
      Object.assign(pages.find(page => page.id === active.id), value.page);
      finish();
      const pageLink = navigation.querySelector(`[data-page="${page.id}"]`);
      if (pageLink) pageLink.textContent = value.page.title;
      showToast("已保存到文档文件");
    } catch (error) {
      message(error.message, true);
    } finally {
      saving = false;
      toolbar.querySelectorAll("button").forEach(button => { button.disabled = false; });
      lesson.inert = false;
    }
  }

  window.FRS_EDITOR = {
    canNavigate() {
      if (!active) return true;
      history.replaceState(null, "", `#${active.id}`);
      message("请先保存或取消编辑");
      return false;
    }
  };

  window.addEventListener("frs:render", editButton);
  lesson.addEventListener("input", () => { if (active) changed(); });
  document.addEventListener("selectionchange", () => { if (active && !linkDialog.open && !imageDialog.open) rememberSelection(); });
  toolbar.addEventListener("mousedown", event => { if (event.target.closest("button")) event.preventDefault(); });
  toolbar.addEventListener("click", event => {
    const button = event.target.closest("[data-format]");
    if (!button || !active) return;
    restoreSelection();
    document.execCommand(button.dataset.format, false);
    rememberSelection();
    changed();
  });
  document.querySelector("#editor-code").onclick = () => {
    const range = restoreSelection();
    const text = range.toString();
    if (!text) { message("请选中需要标记为代码的文字"); return; }
    insertHTML(`<code>${escapeText(text)}</code>`);
  };
  document.querySelector("#editor-link").onclick = () => {
    rememberSelection();
    linkDialog.querySelector("form").reset();
    linkDialog.showModal();
  };
  linkDialog.addEventListener("close", () => {
    if (linkDialog.returnValue !== "insert") return;
    const url = document.querySelector("#editor-link-url").value.trim();
    if (!/^(https?:\/\/|mailto:|#)/i.test(url)) { message("请填写网页地址或章节链接", true); return; }
    const range = restoreSelection();
    insertHTML(`<a href="${escapeText(url).replaceAll('"', '&quot;')}">${escapeText(range.toString() || url)}</a>`);
  });
  document.querySelector("#editor-image").onclick = () => {
    rememberSelection();
    imageDialog.querySelector("form").reset();
    imageDialog.showModal();
  };
  imageDialog.addEventListener("close", async () => {
    if (imageDialog.returnValue !== "insert") return;
    const file = document.querySelector("#editor-image-file").files[0];
    if (!file || file.size > 8 * 1024 * 1024) { message("请选择不超过 8 MB 的图片", true); return; }
    const caption = document.querySelector("#editor-image-caption").value.trim();
    try {
      const data = await new Promise((resolve, reject) => {
        const reader = new FileReader();
        reader.onload = () => resolve(reader.result);
        reader.onerror = () => reject(new Error("图片读取失败"));
        reader.readAsDataURL(file);
      });
      if (!active) return;
      insertHTML(`<figure><img src="${data}" alt="${escapeText(caption).replaceAll('"', '&quot;')}">${caption ? `<figcaption>${escapeText(caption)}</figcaption>` : ""}</figure><p><br></p>`);
    } catch (error) { message(error.message, true); }
  });
  lesson.addEventListener("paste", event => {
    if (!active || !event.target.closest('[contenteditable="true"]')) return;
    event.preventDefault();
    const text = event.clipboardData.getData("text/plain");
    document.execCommand("insertText", false, text);
    changed();
  });
  lesson.addEventListener("dragover", event => { if (active) event.preventDefault(); });
  lesson.addEventListener("drop", event => { if (active) { event.preventDefault(); message("请通过插图按钮插入图片"); } });
  lesson.addEventListener("click", event => {
    if (active && event.target.closest("a")) event.preventDefault();
  });
  document.querySelector("#editor-cancel").onclick = async () => {
    if (saving) return;
    try {
      const value = await request(`/__editor/pages/${active.id}`);
      Object.assign(pages.find(page => page.id === active.id), value.page);
      finish();
    } catch (error) { message(error.message, true); }
  };
  saveButton.onclick = save;
  document.addEventListener("keydown", event => {
    if (active && (event.metaKey || event.ctrlKey) && event.key.toLowerCase() === "s") {
      event.preventDefault();
      save();
    }
  });
  window.addEventListener("beforeunload", event => {
    if (active && dirty) { event.preventDefault(); event.returnValue = ""; }
  });
  if (window.FRS_EDITOR_ENABLED && /^https?:$/.test(location.protocol)) {
    fetch("/__editor/session").then(response => response.ok ? response.json() : null).then(value => {
      if (value && value.token) { session = value; editButton(); }
    }).catch(() => {});
  }
})();
