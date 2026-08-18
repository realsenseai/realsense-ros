# bench/ — temporary benchmarking scratch space

Not part of the wrapper and **not meant to ship**: this directory is a holding place for the
NITROS zero-copy measurement harness while that work is in progress (RSDEV-6254). Move it out or
delete it once the numbers are settled.

`nitros_zc_bench/` carries a `COLCON_IGNORE` so a workspace that has this repo in its `src/` does
not try to build it — and so it cannot collide with the copy that is checked out directly as
`<workspace>/src/nitros_zc_bench` on the Jetson test rig. To use it there, sync the contents
(minus `COLCON_IGNORE`) into that location:

```bash
rsync -a --exclude COLCON_IGNORE bench/nitros_zc_bench/ \
    <workspace>/src/nitros_zc_bench/
```

See `nitros_zc_bench/README.md` for what it measures and how to run it.
