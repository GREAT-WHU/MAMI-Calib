# Third-Party Notices

This repository is derived from and contains code from the following projects.

## MI-Calib

- Upstream: <https://github.com/Unsigned-Long/MI-Calib>
- Repository license file: MIT
- Copyright: Shuolong Chen and the School of Geodesy and Geomatics, Wuhan University

Many MI-Calib source files also carry BSD-style redistribution terms in their file headers. Those embedded notices and conditions remain in force for the corresponding files.

## CTraj

- Upstream: <https://github.com/Unsigned-Long/CTraj>
- Included as: `thirdparty/ctraj`
- Source files carry BSD-style redistribution terms and their original copyright notices.

The non-uniform spline extension modifies CTraj. A public release must publish the exact modified CTraj commit referenced by the parent repository, or vendor it in a license-compliant way.

## tiny-viewer

- Upstream: <https://github.com/Unsigned-Long/tiny-viewer>
- Included through CTraj as: `thirdparty/ctraj/thirdparty/tiny-viewer`
- Retains its upstream copyright and license terms.

## Other dependencies

ROS, Ceres Solver, Sophus, Pangolin, Eigen, PCL, magic_enum, cereal, yaml-cpp, spdlog, and their transitive dependencies are not relicensed by this repository. Consult the installed or upstream packages for their terms.

This notice is informational and is not legal advice. Before publishing, the maintainer should complete a license review of the precise commits being distributed.
