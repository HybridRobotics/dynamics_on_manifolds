package hybridrobotics.dynamics.data_types

trait SmoothManifold {
  def getVariationVector: Any
  def getTangentVector: Any
}

trait SpecialEuclidean {
  // SO(2), SO(3)
}

trait SnManifold {
  // S1, S2, S3 ...
}
