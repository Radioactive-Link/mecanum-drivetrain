{
  description = "Flake for development and ci";
  inputs = {
    nixpkgs.url = "github:nixos/nixpkgs";
    treefmt-nix.url = "github:numtide/treefmt-nix";
  };

  outputs = inputs @ {
    self,
    nixpkgs,
    flake-utils,
    treefmt-nix,
  }:
    flake-utils.lib.eachDefaultSystem
    (
      system: let
        pkgs = inputs.nixpkgs.legacyPackages.${system};
        treefmt = treefmt-nix.lib.evalModule pkgs ./treefmt.nix;
      in {
        packages.default = pkgs.hello;
        # Run "nix develop" in order to use the dev shell
        devShells.default = pkgs.mkShell {
          name = "wpilib";
          nativeBuildInputs = with pkgs; [
            jdk17
            gradle
            jdt-language-server
          ];
          shellHook = ''
            export JAVA_HOME=${pkgs.jdk17}
            alias tasks="sudo gradle tasks"
            alias build="sudo gradle build --build-cache --no-daemon"
            alias test="sudo gradle test"
            alias deploy="sudo gradle deploy --no-daemon"
          '';
        };
        # Ran with "nix fmt"
        formatter = treefmt.config.build.wrapper;
      }
    );
}
