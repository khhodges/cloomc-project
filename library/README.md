# CLOOMC Shared Abstraction Library (Mum Tunnel)

This directory contains shared Church Machine abstractions published by the community.

## Format

Each abstraction is stored as a JSON file under a language subdirectory:

```
library/
  english/
    hello-world.json
    fibonacci.json
  javascript/
    counter.json
  haskell/
    church-numerals.json
  ada/
    symbolic-add.json
  assembly/
    blinky.json
```

## JSON Structure

Each `.json` file contains a complete abstraction upload package:

```json
{
  "name": "hello-world",
  "language": "english",
  "source": "... source code ...",
  "compiled": { ... compiler output ... },
  "doc": {
    "author": "Student Name",
    "date": "2026-03-06",
    "language": "english",
    "description": "A simple hello world program",
    "tags": ["beginner", "tutorial"],
    "methods": [...],
    "capabilities": [...],
    "sourcePreview": "..."
  }
}
```

## Contributing

Abstractions published here are licensed under GPL-3.0, consistent with the
rest of

 the CLOOMC platform. By publishing an abstraction to this library,
you agree to release it under the same license.

### Publishing from the IDE

1. Write and compile your program in the CLOOMC IDE
2. Open the Library modal (click "Library" in the toolbar)
3. Click "Publish" and fill in the details
4. Your abstraction will be pushed to this directory automatically

### Publishing manually

Create a JSON file following the format above and submit a pull request.

## Browsing

You can browse available abstractions:
- **In the IDE:** Open the Library modal and search/filter by language
- **On GitHub:** Browse the subdirectories above
- **Via API:** `GET /api/library/browse` on any running CLOOMC instance

---

*Part of the [CLOOMC Educational Platform](https://github.com/khhodges/cloomc-project) — free and open source under GPL-3.0.*
