# NUMERICS RULES
- Quaternion: Hamilton, [w,x,y,z] internally, body->world. Written as qx qy qz qw.
- Renormalise q every step. Keep sign continuity of the output quaternion.
- Covariance: symmetrise P after every predict and update; floor the diagonal at 1e-12;
  cap at 1e6. Use Joseph form for updates.
- dt <= 0 or dt > gap_threshold: never integrate raw IMU values across it (see gap handler).
- Never divide by a norm without a floor (1e-12).
- Timestamps: keep int64 ns as the master clock. dt = (t_k - t_{k-1}) * 1e-9.
