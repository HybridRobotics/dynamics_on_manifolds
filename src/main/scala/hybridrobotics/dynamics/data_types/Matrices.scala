package hybridrobotics.dynamics.data_types

// Matrix Expression
trait MatrixExpr extends Variable with TimeVarying {
  //Wrap s:String to MatrixExp

  import language.implicitConversions

  implicit def str2MatrixExpr(s: String, t: String): MatrixExpr = Matrix(s)

  override val name: String = ""

  override val size = List(3, 3)

  override def diff(): MatrixExpr = {
    this match {
      case SMMul(u: MatrixExpr, v: ScalarExpr) => MAdd(SMMul(u.diff(), v), SMMul(u, v.diff()))
      case MMul(u: MatrixExpr, v: MatrixExpr) => MAdd(MMul(u.diff(), v), MMul(u, v.diff()))
      case MAdd(u: MatrixExpr, v: MatrixExpr) => MAdd(u.diff(), v.diff())
      case DeltaM(m: MatrixExpr) => DeltaM(m.diff())
      case TransposeMatrix(m: MatrixExpr) => TransposeMatrix(m.diff())
      case CrossMap(v: VectorExpr) => CrossMap(v.diff())
      case _ => this.diff()
    }
  }

  override def delta(): MatrixExpr = DeltaM(this)

  // Algebra (Infix) operators
  def *(u: ScalarExpr): MatrixExpr = SMMul(this, u)

  def **(v: VectorExpr): VectorExpr = MVMul(this, v)

  def ***(m: MatrixExpr): MatrixExpr = MMul(this, m)

  def +(m: MatrixExpr): MatrixExpr = MAdd(this, m)

  def -(m: MatrixExpr): MatrixExpr = MAdd(this, SMMul(m, Num(-1)))

  def T: MatrixExpr = TransposeMatrix(this)

  def det(): ScalarExpr = Det(this)

}


// Matrix Algebra classes
case class SMMul(u: MatrixExpr, v: ScalarExpr) extends MatrixExpr // u *   v infix

case class MVMul(u: MatrixExpr, v: VectorExpr) extends VectorExpr // u **  v infix

case class MMul(u: MatrixExpr, v: MatrixExpr) extends MatrixExpr // u *** v infix

case class MAdd(u: MatrixExpr, v: MatrixExpr) extends MatrixExpr // u +   v infix

case class Det(u: MatrixExpr) extends ScalarExpr // Determinant prefix

case class DeltaM(m: MatrixExpr) extends MatrixExpr // delta prefix

case class TransposeMatrix(m: MatrixExpr) extends MatrixExpr {

  override def T: MatrixExpr = this.m

}

case class CrossMap(v: VectorExpr) extends MatrixExpr with SkewSymmetricMatrix


// Matrix Types
case class Matrix(override val name: String) extends MatrixExpr {

  override def diff(): MatrixExpr = Matrix("dot" + this.name)

}

case class SymMatrix(override val name: String) extends MatrixExpr with SymmetricMatrix {

  override def diff(): MatrixExpr = Matrix("dot" + this.name)

}

case class ConstSymMatrix(override val name: String) extends MatrixExpr with ConstantMatrix with SymmetricMatrix {

  override def diff(): MatrixExpr = SMMul(Matrix("dot" + this.name), Num(0.0))

}

case class ConstMatrix(override val name: String) extends MatrixExpr with ConstantMatrix {

  override def diff(): MatrixExpr = SMMul(Matrix("dot" + this.name), Num(0.0))

}

case class SkewSymMatrix(override val name: String) extends MatrixExpr with SkewSymmetricMatrix {

  override def diff(): MatrixExpr = Matrix("dot" + this.name)

}

case class SO3(override val name: String) extends MatrixExpr with SpecialEuclidean with SmoothManifold {

  val variationStr: String = "eta_{" + name + "}"
  val tangentStr: String = "Omega_{" + name + "}"

  override def det(): ScalarExpr = Num(1.0)

  override def getTangentVector: VectorExpr = Vector(this.tangentStr)

  override def getVariationVector: VectorExpr = Vector(this.variationStr)

  override def delta(): MatrixExpr = MMul(this, CrossMap(this.getVariationVector))

  override def diff(): MatrixExpr = MMul(this, CrossMap(this.getTangentVector))

}