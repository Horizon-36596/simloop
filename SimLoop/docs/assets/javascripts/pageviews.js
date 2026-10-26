// Page-view counter for this docs site (GoatCounter). Horizon (FTC 36596).
//
// No cookies, no local storage, no personal data. GoatCounter records the page path, the referring
// site, the browser and screen size, and a country derived from the IP address, which it then drops.
// The privacy page (privacy.md) says the same; change both together.
//
// Both docs sites (libraries.horizon36596.org/zenith/ and /simloop/) report to ONE GoatCounter site,
// told apart by path. Until the owner creates that site, GOATCOUNTER_CODE is a placeholder and this
// file does nothing.
(function () {
  var GOATCOUNTER_CODE = "REPLACE_WITH_GOATCOUNTER_CODE";
  if (GOATCOUNTER_CODE.indexOf("REPLACE_") === 0) return;
  // Local previews (mkdocs serve) are not counted.
  if (location.hostname !== "libraries.horizon36596.org") return;
  window.goatcounter = { endpoint: "https://" + GOATCOUNTER_CODE + ".goatcounter.com/count" };
  var s = document.createElement("script");
  s.async = true;
  s.src = "https://gc.zgo.at/count.js";
  document.head.appendChild(s);
})();
