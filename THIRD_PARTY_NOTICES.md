# Third-party acknowledgements

This repository uses external open-source software and retains the original contributor and license declarations of the included perception packages. The complete pipeline is not claimed as an original implementation by this repository owner.

## Patchwork++

- Upstream project: [url-kaist/patchwork-plusplus](https://github.com/url-kaist/patchwork-plusplus).
- Role: LiDAR ground segmentation, consumed through the `patchworkpp` ROS package.
- Root copyright in the locally used source: **2024 Urban Robotics Lab. @ KAIST**.
- The local ROS wrapper also contains a MIT notice crediting **Ignacio Vizzo, Tiziano Guadagnino, Benedikt Mersch, and Cyrill Stachniss (2022)**.
- Patchwork++ source is an **external dependency**, not included in this perception-only upload.
- The initial local snapshot has a BSD-2-Clause root LICENSE, a MIT `ros/LICENSE`, and a GPL-3.0 declaration in `ros/package.xml`. Preserve the original notices and determine the applicable file scopes when preparing a distributable modified copy.
- Exact upstream revision of that local snapshot is not yet established. An arbitrary current upstream checkout is not claimed to reproduce it.
- No Patchwork++ algorithm modifications were made in this source export. Future modifications will be recorded separately from the original upstream work.

## Included perception packages

Original package.xml maintainer/license declarations, source notices and READMEs are retained. DBSCAN lists Junsuk Kim; L-shape lists peacon; filtering/tracking contain placeholder maintainer declarations; the integration wrapper lists youngwoo. These declarations are preserved provenance, not a claim that any one maintainer wrote every file. Package-local declarations are retained rather than replaced with a repository-wide license.

The perception wrapper currently contains a TODO license declaration. This document does not invent or replace that declaration. Waypoint datasets, vehicle logs, calibration assets and the rest of the team stack are not part of this upload.
