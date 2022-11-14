# sky 360 Fisheye camera 

# development

the current development is in nixpkgs. the `indi-full` has an issue to support the build of the QHY camera driver. 

For more information follow the issue: https://github.com/NixOS/nixpkgs/pull/200472

# install indi-full runtime

```nix
nix-shell -p indi-full
```
# setup fisheye camera SD Image

TBD. Currently only basic `configuration.nix` is provided for testing purposes, this should be rebuild on a nixos. Future will include SD Images builds for `aarch64` and `x86_64` architectures

## login
the default username is `sky360` and the default password is `sky360`
