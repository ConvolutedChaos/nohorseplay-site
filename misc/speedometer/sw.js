/* Speedo service worker.
 * First visit: precache everything below, so the next launch works with no connection.
 * After that: answer from cache instantly, and refresh the cache in the background
 * (stale-while-revalidate). Edits you deploy show up on the second reload.
 * Bump VERSION when you add or rename files so old caches get cleared. */
const VERSION = 'speedo-v1';
const ASSETS = [
  './',
  './index.html',
  './style.css',
  './app.js',
  './manifest.json',
  './fonts/b612-400.woff2',
  './fonts/b612-700.woff2',
  './icons/icon.svg',
  './icons/icon-192.png',
  './icons/icon-512.png',
  './icons/maskable-192.png',
  './icons/maskable-512.png',
  './icons/apple-touch-icon.png',
  './icons/favicon-32.png',
];

self.addEventListener('install', (event) => {
  event.waitUntil(
    caches.open(VERSION)
      .then((cache) => cache.addAll(ASSETS.map((u) => new Request(u, { cache: 'reload' }))))
      .then(() => self.skipWaiting())
  );
});

self.addEventListener('activate', (event) => {
  event.waitUntil(
    caches.keys()
      .then((keys) => Promise.all(keys.filter((k) => k !== VERSION).map((k) => caches.delete(k))))
      .then(() => self.clients.claim())
  );
});

self.addEventListener('fetch', (event) => {
  const req = event.request;
  if (req.method !== 'GET') return;
  const url = new URL(req.url);
  if (url.origin !== self.location.origin) return;

  const isPage = req.mode === 'navigate';
  event.respondWith((async () => {
    const cache = await caches.open(VERSION);
    const cached = isPage
      ? (await cache.match(req, { ignoreSearch: true })) || (await cache.match('./index.html'))
      : await cache.match(req);

    const network = fetch(req)
      .then((res) => {
        if (res && res.ok && res.type === 'basic') cache.put(isPage ? './index.html' : req, res.clone());
        return res;
      })
      .catch(() => null);

    if (cached) {
      event.waitUntil(network);   // refresh quietly for next time
      return cached;
    }
    const res = await network;
    return res || new Response('Offline, and this file was never cached.', { status: 503, headers: { 'Content-Type': 'text/plain' } });
  })());
});
