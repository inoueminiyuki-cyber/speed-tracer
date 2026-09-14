// Speed Tracer PWA service worker
// キャッシュ名はアプリのバージョン(footer表記)と合わせて更新すること。
// 更新するたびにCACHE_NAMEの数字を変えないと、古いキャッシュが使われ続けます。
const CACHE_NAME = 'speed-tracer-cache-v2026-09-13.33';
const APP_SHELL = [
  './speed_tracer_pc.html',
  './manifest.json',
  './icon-192.png',
  './icon-512.png',
  './icon-maskable-512.png'
];

self.addEventListener('install', (event) => {
  event.waitUntil(
    caches.open(CACHE_NAME).then((cache) => cache.addAll(APP_SHELL))
  );
  self.skipWaiting();
});

self.addEventListener('activate', (event) => {
  event.waitUntil(
    caches.keys().then((names) =>
      Promise.all(
        names.filter((n) => n !== CACHE_NAME).map((n) => caches.delete(n))
      )
    )
  );
  self.clients.claim();
});

// ネットワーク優先、失敗時はキャッシュにフォールバック。
// (ESP32のWiFi AP接続など、通信先そのものは fetch では扱わずアプリ内のWebSocket/BLE/Serialが担当するので、
//  ここで対象にするのはHTML/アイコン/マニフェストなどの「アプリの外側」のみ)
self.addEventListener('fetch', (event) => {
  if (event.request.method !== 'GET') return;
  event.respondWith(
    fetch(event.request)
      .then((res) => {
        const resClone = res.clone();
        caches.open(CACHE_NAME).then((cache) => cache.put(event.request, resClone));
        return res;
      })
      .catch(() => caches.match(event.request))
  );
});
