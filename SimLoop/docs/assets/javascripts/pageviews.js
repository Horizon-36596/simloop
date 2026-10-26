// Page-view counter for this docs site. Horizon (FTC 36596).
//
// It counts with GoatCounter, but runs no GoatCounter code: this file sends one image request to
// GoatCounter's documented /count endpoint (goatcounter.com/help/pixel). No third-party script is
// loaded on purpose: these pages share an origin with the other sites under
// libraries.horizon36596.org, the Zenith web editor (/zenith/app/) among them, so a third-party
// script here could read their browser storage.
//
// No cookies, no local storage. Sent: the page path and title, the referring page, and the screen
// size. GoatCounter also sees the IP address and browser, as any web request does; it uses them to
// work out the country and to count a reload once, keeps them in memory for at most 8 hours and does
// not store them. The privacy page (privacy.md) says the same; change both together.
//
// Both docs sites (libraries.horizon36596.org/zenith/ and /simloop/) report to ONE GoatCounter site,
// told apart by path. Until the owner creates that site, GOATCOUNTER_CODE is a placeholder and this
// file does nothing.
(function () {
  var GOATCOUNTER_CODE = "REPLACE_WITH_GOATCOUNTER_CODE";
  if (GOATCOUNTER_CODE.indexOf("REPLACE_") === 0) return;
  // Local previews (mkdocs serve) are not counted.
  if (location.hostname !== "libraries.horizon36596.org") return;
  if (document.visibilityState === "prerender") return;
  var q = [
    "p=" + encodeURIComponent(location.pathname),
    "t=" + encodeURIComponent(document.title),
    "r=" + encodeURIComponent(document.referrer),
    "s=" + [screen.width, screen.height, window.devicePixelRatio || 1].join(","),
    "rnd=" + Math.random().toString(36).slice(2),
  ];
  // 153 is GoatCounter's "WebDriver-based headless browser" bot flag.
  if (navigator.webdriver) q.push("b=153");
  new Image().src = "https://" + GOATCOUNTER_CODE + ".goatcounter.com/count?" + q.join("&");
})();
