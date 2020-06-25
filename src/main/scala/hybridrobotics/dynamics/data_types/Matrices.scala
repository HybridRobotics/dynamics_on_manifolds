package hybridrobotics.dynamics.data_types

// Matrix Expression
trait MatrixExpr extends Expression with TimeVarying {
  //Wrap s:String to MatrixExp

  import language.implicitConversions

  implicit def str2MatrixExpr(s: String, t: String): MatrixExpr = Matrix(s)

  override def diff(): MatrixExpr = {
    this match {
      case SMMul(m: MatrixExpr, s: ScalarExpr) =>
        if (s.isInstanceOf[NumScalar] || s.isInstanceOf[ConstScalar]) SMMul(m.diff(), s)
        else if (m.isInstanceOf[ConstantMatrix]) SMMul(m, s.diff())
        else MAdd(SMMul(m.diff(), s), SMMul(m, s.diff()))

      case MMul(a: MatrixExpr, b: MatrixExpr) =>
        if (a.isInstanceOf[ConstantMatrix]) MMul(a, b.diff())
        else if (b.isInstanceOf[ConstantMatrix]) MMul(a.diff(), b)
        else MAdd(MMul(a.diff(), b), MMul(a, b.diff()))

      case MAdd(a: MatrixExpr, b: MatrixExpr) =>
        if (a.isInstanceOf[ConstantMatrix]) b.diff()
        else if (b.isInstanceOf[ConstantMatrix]) a.diff()
        else MAdd(a.diff(), b.diff())

      case DeltaM(m: MatrixExpr) => DeltaM(m.diff())

      case TransposeMatrix(m: MatrixExpr) => TransposeMatrix(m.diff())

      case CrossMap(v: VectorExpr) => CrossMap(v.diff())

      case _ => this.diff()
    }
  }

  override def delta(): MatrixExpr = {
    this match {
      case SMMul(m: MatrixExpr, s: ScalarExpr) =>
        if (s.isInstanceOf[NumScalar] || s.isInstanceOf[ConstScalar]) SMMul(m.delta(), s)
        else if (m.isInstanceOf[ConstantMatrix]) SMMul(m, s.delta())
        else MAdd(SMMul(m.delta(), s), SMMul(m, s.delta()))

      case MMul(a: MatrixExpr, b: MatrixExpr) =>
        if (a.isInstanceOf[ConstantMatrix]) MMul(a, b.delta())
        else if (b.isInstanceOf[ConstantMatrix]) MMul(a.delta(), b)
        else MAdd(MMul(a.delta(), b), MMul(a, b.delta()))

      case MAdd(a: MatrixExpr, b: MatrixExpr) =>
        if (a.isInstanceOf[ConstantMatrix]) b.delta()
        else if (b.isInstanceOf[ConstantMatrix]) a.delta()
        else MAdd(a.delta(), b.delta())

      case TransposeMatrix(m: MatrixExpr) => TransposeMatrix(m.delta())

      case CrossMap(v: VectorExpr) => CrossMap(v.delta())

      case _ => this.delta()
    }
  }

  override def getVariation: Any = this.delta() // TODO fix this datatype dependency issue

  // is a element in expression Expr
  def is_member(element: Any): Boolean = {
    // this function verifies if a given vector is part of a expression
    this match {
      // Matrix Expr
      case MAdd(a, b) => a.is_member(element) || b.is_member(element)
      case MMul(u, v) => u.is_member(element) || v.is_member(element)
      case SMMul(m, s) => m.is_member(element) || s.is_member(element)
      case VVMul(a, b) => a.is_member(element) || b.is_member(element)
      case TransposeMatrix(m) => m.is_member(element)
      case CrossMap(v) => v.is_member(element)
      case _ =>
        if (this == element)
          return true
        else
          return false
    }
  }

  // Algebra (Infix) operators
  def *(u: ScalarExpr): MatrixExpr = SMMul(this, u)

  def **(v: VectorExpr): VectorExpr = MVMul(this, v)

  def ***(m: MatrixExpr): MatrixExpr = MMul(this, m)

  def +(m: MatrixExpr): MatrixExpr = MAdd(this, m)

  def -(m: MatrixExpr): MatrixExpr = MAdd(this, SMMul(m, NumScalar(-1)))

  def T: MatrixExpr = TransposeMatrix(this)

  def det(): ScalarExpr = Det(this)

  def d: MatrixExpr = this

  // other functions
  override def basicSimplify(): MatrixExpr = {
    this match {

      case _ => this
    }
  }

}

//
// Matrix Algebra classes
//
case class SMMul(u: MatrixExpr, v: ScalarExpr) extends MatrixExpr {
  // Matrix Scalar Multiplication
  // u *   v infix
  override def basicSimplify(): MatrixExpr = {
    this match {
      case _ => SMMul(u.basicSimplify(), v.basicSimplify())
    }
  }

}

case class MVMul(u: MatrixExpr, v: VectorExpr) extends VectorExpr {
  // Matrix Vector Multiplication
  // u **  v infix

  // simplify function
  override def basicSimplify(): VectorExpr = {
    this match {
      case MVMul(u, v) => MVMul(u.basicSimplify(), v.basicSimplify())
      case _ => this
    }
  }

}

case class MMul(u: MatrixExpr, v: MatrixExpr) extends MatrixExpr {
  // Matrix Multiplication
  // u *** v infix

  // simplify function
  override def basicSimplify(): MatrixExpr = {
    this match {
      case MMul(IdentityMatrix(), v) => v
      case MMul(u, IdentityMatrix()) => u
      case MMul(u, ZeroMatrix()) => ZeroMatrix()
      case MMul(ZeroMatrix(), v) => ZeroMatrix()
      case MMul(u, v) => MMul(u.basicSimplify(), v.basicSimplify())
      case _ => this
    }
  }
}

case class MAdd(u: MatrixExpr, v: MatrixExpr) extends MatrixExpr {
  // Matrix Addition
  // u +  v infix

  // simplify function
  override def basicSimplify(): MatrixExpr = {
    this match {
      // TODO update this crude simplification
      case MAdd(ZeroMatrix(), v) => v
      case MAdd(u, ZeroMatrix()) => u

      case MAdd(SMMul(u, s1), SMMul(v, s2)) =>
        if (u == v) SMMul(v.basicSimplify(), Add(s1, s2).basicSimplify())
        else MAdd(SMMul(u.basicSimplify(), s1.basicSimplify()), SMMul(v.basicSimplify(), s2.basicSimplify()))

      case MAdd(SMMul(u, s), v) => MAdd(v, SMMul(u, s)).basicSimplify()

      case MAdd(u, SMMul(v, s)) =>
        if (u == v) MAdd(SMMul(u, NumScalar(1.0)), SMMul(v, s)).basicSimplify()
        else MAdd(u.basicSimplify(), SMMul(v.basicSimplify(), s.basicSimplify()))

      case MAdd(u, v) =>
        if (u == v) SMMul(u, NumScalar(2.0))
        else MAdd(u.basicSimplify(), v.basicSimplify())
      case MAdd(u, SMMul(v, NumScalar(-1.0))) =>
        if (u == v) SMMul(u, NumScalar(0.0))
        else MAdd(u.basicSimplify(), SMMul(v.basicSimplify(), NumScalar(-1.0)))

      case _ => MAdd(u.basicSimplify(), v.basicSimplify())
    }
  }
}

case class Det(u: MatrixExpr) extends ScalarExpr // Determinant prefix

case class DeltaM(m: MatrixExpr) extends MatrixExpr // delta prefix

case class TransposeMatrix(m: MatrixExpr) extends MatrixExpr {

  override def T: MatrixExpr = this.m

  override def basicSimplify(): MatrixExpr = {
    this match {
      case TransposeMatrix(MAdd(u, v)) => MAdd(TransposeMatrix(u.basicSimplify()).basicSimplify(), TransposeMatrix(v.basicSimplify()).basicSimplify())
      case TransposeMatrix(MMul(u, v)) => MMul(TransposeMatrix(v.basicSimplify()).basicSimplify(), TransposeMatrix(u.basicSimplify()).basicSimplify())
      case TransposeMatrix(CrossMap(v)) => SMMul(CrossMap(v.basicSimplify()), NumScalar(-1.0))
      case TransposeMatrix(m) => TransposeMatrix(m.basicSimplify())
      case _ => this
    }
  }

}

case class CrossMap(v: VectorExpr) extends MatrixExpr with SkewSymmetricMatrix {
  override def basicSimplify(): MatrixExpr = {
    this match {
      case _ => CrossMap(v.basicSimplify())
    }
  }
}

case class MatrixSummation(matrix: MatrixExpr) extends MatrixExpr with Iterator {
  val index: String = "i"
  val from: String = "1"
  val to: String = "n"

  override def delta(): MatrixExpr = {
    MatrixSummation(matrix.delta())
  }

  override def diff(): MatrixExpr = {
    MatrixSummation(matrix.diff())
  }

  override def basicSimplify(): MatrixExpr = {
    MatrixSummation(matrix.basicSimplify())
  }
}

//
// Matrix Types
//
trait BaseMatrixVariable extends MatrixExpr with Variable {

  override val size = List(3, 3)

  override def diff(): MatrixExpr = Matrix("dot" + this.name)

  override def delta(): MatrixExpr = DeltaM(this)

  override def d: MatrixExpr = Matrix(this.name + "_d")

  override def getVariation: Any = this.delta()
}

case class Matrix(override val name: String) extends BaseMatrixVariable {
  override def getVariation: MatrixExpr = this.delta()
}

case class SymMatrix(override val name: String) extends BaseMatrixVariable with SymmetricMatrix {

  override def d: MatrixExpr = SymMatrix(this.name + "_d")

  override def getVariation: MatrixExpr = this.delta()

}

case class ConstSymMatrix(override val name: String) extends BaseMatrixVariable with ConstantMatrix with SymmetricMatrix {

  override def diff(): MatrixExpr = SMMul(this, NumScalar(0.0))

  override def delta(): MatrixExpr = SMMul(this, NumScalar(0.0))

  override def d: MatrixExpr = this

  override def getVariation: MatrixExpr = this.delta()

}

case class ConstMatrix(override val name: String) extends BaseMatrixVariable with ConstantMatrix {

  override def diff(): MatrixExpr = SMMul(this, NumScalar(0.0))

  override def delta(): MatrixExpr = SMMul(this, NumScalar(0.0))

  override def d: MatrixExpr = this

  override def getVariation: MatrixExpr = this.delta()

}

case class IdentityMatrix() extends BaseMatrixVariable with ConstantMatrix with SymmetricMatrix {

  override val name: String = "IdentityMatrix"

  override def diff(): MatrixExpr = SMMul(this, NumScalar(0.0))

  override def delta(): MatrixExpr = SMMul(this, NumScalar(0.0))

  override def d: MatrixExpr = this

  override def getVariation: MatrixExpr = this.delta()

}

case class ZeroMatrix() extends BaseMatrixVariable with ConstantMatrix with SymmetricMatrix {

  override val name: String = "ZeroMatrix"

  override def diff(): MatrixExpr = SMMul(this, NumScalar(0.0))

  override def delta(): MatrixExpr = SMMul(this, NumScalar(0.0))

  override def d: MatrixExpr = this

  override def getVariation: MatrixExpr = this.delta()

}


case class SkewSymMatrix(override val name: String) extends BaseMatrixVariable with SkewSymmetricMatrix {

  override def d: MatrixExpr = SkewSymMatrix(this.name + "_d")

  override def getVariation: MatrixExpr = this.delta()

}

case class SO3(override val name: String) extends BaseMatrixVariable with SpecialEuclidean with SmoothManifold {

  val variationStr: String = "eta_{" + name + "}"
  val tangentStr: String = "Omega_{" + name + "}"

  override def det(): ScalarExpr = NumScalar(1.0)

  override def getTangentVector: VectorExpr = Vector(this.tangentStr)

  override def getVariationVector: VectorExpr = Vector(this.variationStr)

  override def delta(): MatrixExpr = MMul(this, CrossMap(this.getVariationVector))

  override def diff(): MatrixExpr = MMul(this, CrossMap(this.getTangentVector))

  override def d: MatrixExpr = SO3(this.name + "_d")

  override def getVariation: VectorExpr = this.getVariationVector

}