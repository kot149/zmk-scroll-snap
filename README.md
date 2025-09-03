# ZMK Scroll Snap Module

A ZMK module that provides a scroll snap feature: align the 360-degree xy scroll direction to the nearest axis.

> [!warning]
> 🚧 This module is still under development. 🚧
>
> Its behavior is not stable. Its behavior and API may change without notice.

## Usage

### 1. Add the Module to your `west.yml`.

```yml
manifest:
  remotes:
    - name: zmkfirmware
      url-base: https://github.com/zmkfirmware
    - name: kot149
      url-base: https://github.com/kot149
  projects:
    - name: zmk
      remote: zmkfirmware
      revision: main
      import: app/west.yml
    - name: zmk-scroll-snap
      remote: kot149
      revision: v1
  self:
    path: config
```

### 2. Include `scroll-snap.dtsi`

```c
#include <scroll-snap.dtsi>
```

### 3. Apply the input processor to your pointing device listener

```dts
#include <scroll-snap.dtsi>

&trackball_listener {
    compatible = "zmk,input-listener";
    device = <&trackball>;

    input-processors = <&zip_scroll_snap>;
};
```

## Configuration

In the following example, the scroll snap will
- snap to x axis if $\displaystyle \left|\frac{y}{x}\right| < \frac{5}{8}$.
- snap to y axis if $\displaystyle \left|\frac{y}{x}\right| > \frac{8}{5}$.
- ⚠️ not implemented: snap to the diagonal line ($y=\pm x$) if $\displaystyle \frac{5}{8} < \left|\frac{y}{x}\right| < \frac{8}{5}$.
- collect 10 samples before start snapping
- if sum of sample value exceeds 1500, start snapping regardless of the number of collected samples
- after snapping, lock direction for 200ms
- after snapping, lock direction for next 10 events
- reset collected samples if idle for 200ms (0 disables)

```dts
&zip_scroll_snap {
    x-threshold = <5 8>;
    y-threshold = <8 5>;
    xy-threshold = <5 8>;

    require-n-samples = <10>;
    immediate-snap-threshold = <1500>;
    lock-duration-ms = <200>;
    lock-for-next-n-events = <10>;
    idle-reset-timeout-ms = <200>;
};
```

See [dts/scroll-snap.dtsi](dts/scroll-snap.dtsi) for default values.