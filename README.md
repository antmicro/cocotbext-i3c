###### Copyright (C) 2024 Antmicro

# cocotbext-i3c

I3C CocoTB simulation models

## I3cController

The `I3cController` class located in `i3c_controller` module can be used to simulate an I3C Host controller.

### Setting up `I3cController`

The `I3cController`  class requires the following signals to be connected to _DUT_:

* `scl_i` - Clock input to controller (output from target(s))
* `sda_i` - Data input to controller (output from target(s))
* `scl_o` - Clock output from of (input of target(s))
* `sda_o` - Data output from of (input to target(s))

The design has to include the physical layer responsible for separating a tri-stat signal
into separate ins and outs.

```python
i3c_controller = I3cController(
    sda_i=dut.sda_o,
    sda_o=dut.sda_i,
    scl_i=dut.scl_o,
    scl_o=dut.scl_i
)
```

The `I3cTimings` class can be used to configure timing parameters of the controller.
Pass an object of this class to the initailizer as `timings` parameter to adjust the timings.
The default values are set to reflect minimal timings on the host controller side, as described by the specification, while allowing the targets to take the maximum allowed time to respond.

Refer to section 6.2 of _MIPI Alliance Specification for I3C Basic Version 1.1.1 (Errata 01)_ for detailed information about timings.

The `speed` parameter of `I3cController` initializer can be used to scale the timings of the device, this includes all the timings defined by `timings` parameter.
The default value, corresponding to 1:1 scale is `12.5e6`, which corresponds to 12.5MHz clocking speed of I3C.

### Using `I3cController`

* `I3cController.i3c_write` - Write data from Host to Target
  * **(default)** Use `mode=I3cXferMode.PRIVATE` for Private I3C Write)
  * Use `mode=I3cXferMode.LEGACY_I2C` for Legacy I2C Write)
* `I3cController.i3c_read` - Read data from Target to Host
  * **(default)** Use `mode=I3cXferMode.PRIVATE` for Private I3C Read)
  * Use `mode=I3cXferMode.LEGACY_I2C` for Legacy I2C Read)
* `I3cController.i3c_ccc_write` - I3C Write CCC
  * For broadcast CCCs set `broadcast_data` parameter to an iterable of bytes
  * For directed CCCs set `directed_data` to an iterable of tuples where
    * First element of the tuple is the address of a device
    * Second element of the tuple is an iterable of bytes - data to send to the addressed device
  * To use a Defining Byte set `defining_byte` argument
* `I3cController.i3c_ccc_read` - I3C Read CCC
  * To use a Defining Byte set `defining_byte` argument

All read/write procedures issue a **STOP** condition on the bus by default.
However it is possible to chain the transfers with repeated start (where applicable).
In order to do it, set the `stop` parameter to `False` on any of the read/write procedures.

Eg.

```python
data0 = await tb.i3c_controller.i3c_read(0x50, 20, stop=False)
data1 = await tb.i3c_controller.i3c_read(0x51)
```
