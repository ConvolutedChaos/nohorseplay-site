# nohorseplay

Welcome to **nohorseplay**! This personal project of mine features some fun things, mostly Unity games. I also recommend that you check out E-Dog OS.

It's a static website (plain HTML, CSS and JavaScript) with no build step.

## Pages

| Page | Description |
| --- | --- |
| [index.html](index.html) | Home |
| [games.html](games.html) | Game listing |
| [tools.html](tools.html) | Tool listing |
| [stuff.html](stuff.html) | Other stuff |
| [changelogs.html](changelogs.html) | Changelogs |
| [privacy.html](privacy.html) | Privacy policy |
| [404.html](404.html) | Not-found page |

## Games ([games/](games/))

Unity games:

- Basketball Simulator
- ESU10
- Lawn Mowing Simulator
- OnEdge
- Physics Fun (V1 and V2)
- Police Simulator
- Store Simulator
- Tornado Simulator
- WebCars (V1, V2 and V3)

Web games and demos:

- Checkers
- Chatbot
- Fluid Sim
- Apple

## Tools ([tools/](tools/))

- Converters
- Decision Maker
- Epoch Time Converter
- GIF Maker
- Image Converter
- Text Editor

## E-Dog OS ([edogos/](edogos/))

A web-based desktop operating system simulation. The older version lives in `misc/edogos-old/`.

## Misc ([misc/](misc/))

Assorted experiments and small pages:

- Code Editor
- macOS and iOS 9 simulators (`macsim/`, `ios9sim/`)
- Speedometer and Speed Test
- Zip Explorer
- News
- HTTP Status Codes
- Potion Index
- Bee Movie script
- Angry Birds and scary galleries

## Project structure

```
css/        Stylesheets (Bootstrap, Font Awesome, site styles)
js/         Scripts (jQuery, Bootstrap, Chartist, shared site scripts)
fonts/      Web fonts
icons/      Icons
img/        Images
games/      Games
tools/      Tools
edogos/     E-Dog OS
misc/       Miscellaneous pages and experiments
```

## Running locally

Serve the folder with any static file server, for example:

```
python -m http.server
```

Then open http://localhost:8000.
