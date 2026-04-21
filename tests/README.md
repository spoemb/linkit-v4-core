# tests/

Host (x86 Linux) unit test suite using [CppUTest](http://cpputest.github.io/).

## Layout

| Folder | Role |
|--------|------|
| [`src/`](src/) | Test files — one per module under test |
| [`mocks/`](mocks/) | CppUTest mocks (call expectations, return-value injection) |
| [`fakes/`](fakes/) | Stub implementations: nRF SDK headers, BSP, fake services |
| `data/` | Fixture data (sample AOP files, configs, packets) |
| `build/` | CMake build output (gitignored) |
| `reports/` | XML test reports |

## Build & run

```bash
cd tests && mkdir -p build && cd build
cmake .. && make -j$(nproc)
cd .. && ln -sf data build/ && ./build/TrackerTests -v
```

Or via the script:
```bash
./scripts/build_unit_tests.sh
```

## Pre-commit hook

`.git/hooks/pre-commit` builds `TrackerTests` on every commit (build only, not run). A failed build aborts the commit.

## Adding a test

1. Add `mything_test.cpp` in [`src/`](src/), using CppUTest's `TEST_GROUP` / `TEST` macros.
2. Add the file to [`CMakeLists.txt`](CMakeLists.txt) `target_sources` for `TrackerTests`.
3. Mock or fake any hardware your code touches (see [`mocks/README.md`](mocks/README.md), [`fakes/README.md`](fakes/README.md)).
