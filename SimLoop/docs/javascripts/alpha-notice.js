// A one-time notice that SimLoop is an alpha, shown the first time someone opens the documentation.
// Horizon (FTC 36596).
//
// Shown once per version: the version is read from the footer (mkdocs.yml `copyright:`), so a new alpha
// shows it again. Dismissing it is remembered in localStorage, which can be missing or blocked (private
// windows, cleared site data); then the notice simply shows on each visit, and the page still works.
(function () {
  "use strict";

  function version() {
    var footer = document.querySelector(".md-copyright");
    var match = footer && footer.textContent.match(/SimLoop\s+(\S+)/);
    return match ? match[1] : "alpha";
  }

  function seen(key) {
    try { return window.localStorage.getItem(key) === "1"; } catch (e) { return false; }
  }

  function remember(key) {
    try { window.localStorage.setItem(key, "1"); } catch (e) { /* nothing to keep it in */ }
  }

  function link(href, text) {
    var a = document.createElement("a");
    a.href = href;
    a.textContent = text;
    return a;
  }

  function show() {
    var v = version();
    var key = "simloop-alpha-notice-" + v;
    if (seen(key) || typeof HTMLDialogElement !== "function") { return; }

    // Material sets __md_scope to the site root, so the link resolves from any page depth.
    var tested = new URL("tested/", window.__md_scope || window.location.href).href;

    var dialog = document.createElement("dialog");
    dialog.className = "simloop-alpha-notice";
    dialog.setAttribute("aria-labelledby", "simloop-alpha-notice-title");

    var title = document.createElement("h2");
    title.id = "simloop-alpha-notice-title";
    title.textContent = "SimLoop " + v + " is an alpha";

    var p1 = document.createElement("p");
    p1.textContent = "It has been used by one team, on one robot's code, and its physics has not yet been " +
      "compared with a real robot. Expect bugs and gaps, and pin an exact version: the API may change " +
      "between alphas.";

    var p2 = document.createElement("p");
    p2.appendChild(document.createTextNode("If you try it, you are testing it. "));
    p2.appendChild(link(tested, "What has been tested"));
    p2.appendChild(document.createTextNode(" says what has been checked, and "));
    p2.appendChild(link("https://github.com/Horizon-36596/simloop/issues", "Issues"));
    p2.appendChild(document.createTextNode(" is where to tell us what you find."));

    var actions = document.createElement("form");
    actions.method = "dialog";
    actions.className = "simloop-alpha-notice__actions";
    var ok = document.createElement("button");
    ok.type = "submit";
    ok.textContent = "Got it";
    actions.appendChild(ok);

    dialog.appendChild(title);
    dialog.appendChild(p1);
    dialog.appendChild(p2);
    dialog.appendChild(actions);
    dialog.addEventListener("close", function () { remember(key); dialog.remove(); });

    document.body.appendChild(dialog);
    dialog.showModal();
    ok.focus();
  }

  if (document.readyState === "loading") {
    document.addEventListener("DOMContentLoaded", show);
  } else {
    show();
  }
})();
