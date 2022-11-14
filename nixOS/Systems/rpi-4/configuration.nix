{ config, pkgs, ... }:

{
  imports =
    [
      ./hardware-configuration.nix
    ];

  boot.loader.grub.enable = false;
  boot.loader.generic-extlinux-compatible.enable = true;
  networking.hostName = "allskycamera";
  networking.networkmanager.enable = true;

  time.timeZone = "America/Chicago";
  i18n.defaultLocale = "en_US.UTF-8";
  console = {
    font = "ter-powerline-v32n";
    useXkbConfig = true;
    packages = [ pkgs.terminus_font pkgs.powerline-fonts ];
  };

  sound.enable = true;
  hardware.pulseaudio.enable = true;
  users.users.sky360 = {
    isNormalUser = true;
    extraGroups = [ "wheel" ];
    initialPassword = "sky360";
    packages = with pkgs; [
      indi-full
    ];
    shell = pkgs.fish;
  };
  
  users.defaultUserShell = pkgs.fish;
   
  environment.sessionVariables = rec {
    TERM="iterm";
  };

  environment.systemPackages = with pkgs; [
    helix rnix-lsp nixpkgs-fmt
    btop
    lsd exa bat wget neofetch mcfly direnv
    ranger mc
    git gitui tig gh
    usbutils
  ];

  programs.fish.enable = true;
  services.openssh.enable = true;
  system.stateVersion = "22.11";
}
