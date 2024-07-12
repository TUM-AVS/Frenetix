{
  inputs = {
    utils.url = "github:numtide/flake-utils";
  };
  outputs = { self, nixpkgs, utils }: utils.lib.eachDefaultSystem (system:
    let
      pkgs = nixpkgs.legacyPackages.${system};
    in
    rec {
      devShells.default = pkgs.mkShell {
        # inputsFrom = [ packages.default ];
        buildInputs = with pkgs; [
          boost
          llvmPackages_18.openmp
          eigen
          taskflow
          spdlog
        ];

        # nativeBuildInputs is usually what you want -- tools you need to run
        nativeBuildInputs = with pkgs; [
          cmake
          clang_18
          gfortran
          clang-tools_18
          # cmake
          (python311.withPackages (ps: with ps; [
            pip
            scikit-build-core
            # nanobind
          ]))
        ];

        dontUseCmakeConfigure = true;


        NIX_LD_LIBRARY_PATH = pkgs.lib.makeLibraryPath [
          pkgs.stdenv.cc.cc
          pkgs.openssl
          # ...
        ];
        # NIX_LD = pkgs.lib.fileContents "${pkgs.stdenv.cc}/nix-support/dynamic-linker";
        NIX_LD = "${pkgs.glibc}/lib/ld-linux-x86-64.so.2";
      };

      packages.default = pkgs.callPackage ./default.nix { };
    }
  );
}

