# tests/mocks/

CppUTest mocks — used when a test needs to **assert on calls** made to a dependency (call counts, parameter matching, return-value injection).

Naming: `mock_<module>.{hpp,cpp}` (e.g. `mock_gpio.cpp`, `mock_battery_mon.hpp`).

Inside the test:
```cpp
mock().expectOneCall("set").withParameter("pin", SAT_PWR_EN);
// run code under test …
mock().checkExpectations();
```

If you only need a do-nothing stub (no behavior assertions), put it in [`../fakes/`](../fakes/) instead — it keeps tests faster and less brittle.
