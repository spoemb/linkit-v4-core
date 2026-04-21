# tests/src/

Unit test source files (one per module under test).

Naming convention: `<module>_test.cpp` — for example `argos_tx_test.cpp`, `sws_analog_test.cpp`, `dte_handler_test.cpp`.

The entry point is `main.cpp`. Each test file declares one or more CppUTest `TEST_GROUP` blocks; CommandLineTestRunner discovers them at link time.

To filter to a specific group at runtime:
```bash
./build/TrackerTests -g ArgosTxService -v
```

When adding a test:
1. Create the file here.
2. Register it in [`../CMakeLists.txt`](../CMakeLists.txt) under `target_sources`.
3. If your module needs a mock or fake, see [`../mocks/`](../mocks/) and [`../fakes/`](../fakes/).
