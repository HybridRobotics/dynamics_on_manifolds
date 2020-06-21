package hybridrobotics.dynamics.data_types
import hybridrobotics.dynamics.data_types.VectorExpr
import hybridrobotics.dynamics.operations.Num

// Scalar Expression
trait ScalarExpr extends Expression with TimeVarying {
  //Wrap s:String to ScalarExpr

  import language.implicitConversions

  implicit def int2ScalarExpr(i: Int): ScalarExpr = NumScalar(i.toDouble)

  implicit def double2ScalarExpr(d: Double): ScalarExpr = NumScalar(d)

  implicit def str2ScalarExpr(name: String): ScalarExpr = VarScalar(name)

  override def diff(): ScalarExpr = {
    this match {
      case DeltaS(u) => DeltaS(u.diff())

      case Par(u) => Par(u.diff())

      case Mul(u, v) =>
        if (u.isInstanceOf[NumScalar] || u.isInstanceOf[ConstScalar]) Mul(u, v.diff())
        else if (v.isInstanceOf[NumScalar] || v.isInstanceOf[ConstScalar]) Mul(u.diff(), v)
        else Mul(u.diff(), v) + Mul(u, v.diff())

      case Add(u, v) =>
        if (u.isInstanceOf[NumScalar] || u.isInstanceOf[ConstScalar]) v.diff()
        else if (v.isInstanceOf[NumScalar] || v.isInstanceOf[ConstScalar]) u.diff()
        else u.diff() + v.diff()

      case Dot(a, b) =>
        if (a.isInstanceOf[ConstantVector] || a.isInstanceOf[ZeroVector]) Dot(a, b.diff())
        else if (b.isInstanceOf[ConstantVector] || b.isInstanceOf[ZeroVector]) Dot(a.diff(), b)
        else Dot(a.diff(), b) + Dot(a, b.diff())

      case _ => this.diff()
    }
  }

  override def delta(): ScalarExpr = {
    this match {
      case Add(u, v) =>
        if (u.isInstanceOf[NumScalar] || u.isInstanceOf[ConstScalar]) v.delta()
        else if (v.isInstanceOf[NumScalar] || v.isInstanceOf[ConstScalar]) u.delta()
        else Add(u.delta(), v.delta())

      case Mul(u, v) =>
        if (u.isInstanceOf[NumScalar] || u.isInstanceOf[ConstScalar]) Mul(u, v.delta())
        else if (v.isInstanceOf[NumScalar] || v.isInstanceOf[ConstScalar]) Mul(u.delta(), v)
        else Mul(u.delta(), v) + Mul(u, v.delta())

      case Dot(a, MVMul(m, b)) =>
        if ((m.isInstanceOf[SymmetricMatrix]) && (a == b)) Add(Mul(NumScalar(2), Dot(a.delta(), MVMul(m, b))), Dot(a, MVMul(m.delta(), b)))
        else Dot(a.delta(), MVMul(m, b)) + Dot(a, MVMul(m, b).delta())

      case Dot(a, b) =>
        if (a.isInstanceOf[ConstantVector] || a.isInstanceOf[ZeroVector]) Dot(a, b.delta())
        else if (b.isInstanceOf[ConstantVector] || b.isInstanceOf[ZeroVector]) Dot(a.delta(), b)
        else Dot(a.delta(), b) + Dot(a, b.delta())

      case _ => this.diff()
    }
  }

  // is a vector element  in expression vectorExpr
  def is_member(element: Any): Boolean = {
    // this function verifies if a given vector is part of a expression
    this match {
      // Scalar Expr
      case Add(a, b) => a.is_member(element) || b.is_member(element)
      case Mul(a, b) => a.is_member(element) || b.is_member(element)
      case _ =>
        if (this == element)
          return true
        else
          return false
    }
  }

  override def getVariation: ScalarExpr = this.delta()

  // (Algebra) Infix operators from high to low using Scala precedence
  def *(v: ScalarExpr): ScalarExpr = Mul(this, v)

  def +(v: ScalarExpr): ScalarExpr = Add(this, v)

  def -(v: ScalarExpr): ScalarExpr = Add(this, Mul(NumScalar(-1), v)) // Subtract is stored as adding a negative

  def T: ScalarExpr = this // scalar transpose

  // other operations
  override def basicSimplify(): ScalarExpr = {
    this match {
      // Dot product using ZeroVector
      case Dot(vector1, vector2) =>
        if (vector1.isInstanceOf[ZeroVector] || vector2.isInstanceOf[ZeroVector]) NumScalar(0.0)
        else Dot(vector1, vector2)

      // If nothing matches return without simplify
      case _ => this
    }

  }

  def extractNumericCoeff(): Double = {
    this match {
      case Mul(u, v) => u.extractNumericCoeff() * v.extractNumericCoeff()
      case NumScalar(d) => d
      case _ => return 1
    }
  }

  def extractScalarExpr(): ScalarExpr = {
    this match {
      case Mul(NumScalar(d), v) => v.extractScalarExpr()
      case Mul(v, NumScalar(d)) => v.extractScalarExpr()
      case Mul(u, v) =>
        if (!u.isInstanceOf[NumScalar] && !v.isInstanceOf[NumScalar]) u.extractScalarExpr() * v.extractScalarExpr()
        else if (!u.isInstanceOf[NumScalar]) u.extractScalarExpr()
        else v.extractScalarExpr()
      case _ => this
    }
  }

}


//
// Scalar Types
//
trait BaseScalarVariable extends ScalarExpr with Variable {
  override val size = List(1, 1)

  override def diff(): ScalarExpr = VarScalar("dot" + this.name)

  override def delta(): ScalarExpr = DeltaS(this)

  override def getVariation: ScalarExpr = this.delta()

  override def d: ScalarExpr = VarScalar(this.name + "_d")
}

case class NumScalar(value: Double) extends BaseScalarVariable {
  // Numeric
  override val name: String = String.valueOf(value)

  override def diff(): ScalarExpr = NumScalar(0.0)

  override def delta(): ScalarExpr = NumScalar(0.0)

  override def d: ScalarExpr = this
}

case class VarScalar(override val name: String) extends BaseScalarVariable

case class ConstScalar(override val name: String) extends BaseScalarVariable {
  // Constant scalar
  override def diff(): ScalarExpr = NumScalar(0.0)

  override def delta(): ScalarExpr = NumScalar(0.0)

  override def d: ScalarExpr = this
}

//
// Scalar Operation classes
//
case class DeltaS(u: ScalarExpr) extends ScalarExpr // delta prefix

case class Par(u: ScalarExpr) extends ScalarExpr // parentheses

case class Mul(u: ScalarExpr, v: ScalarExpr) extends ScalarExpr {
  // u * v infix
  override def basicSimplify(): ScalarExpr = {
    this match {
      // Multiply Numerals
      case Mul(NumScalar(number1), NumScalar(number2)) => NumScalar(number1 * number2)

      // Extract Coefficients
      case Mul(u, v) => Mul(NumScalar(Mul(u, v).extractNumericCoeff()), Mul(u, v).extractScalarExpr())
      case _ => Mul(u.basicSimplify(), v.basicSimplify())
    }
  }


}

case class Add(u: ScalarExpr, v: ScalarExpr) extends ScalarExpr {
  // u + v infix

  override def basicSimplify(): ScalarExpr = {
    Add(u, v) match {
      // Add Numerals
      case Add(NumScalar(number_1), NumScalar(number_2)) => NumScalar(number_1 + number_2)

      // Remove Zero Numerals
      case Add(Mul(a1, NumScalar(d1)), Mul(a2, NumScalar(d2))) =>
        if (d1 == 0 && d2 == 0)
          return NumScalar(0.0)
        else
          Add(Mul(a1.basicSimplify(), NumScalar(d1)), Mul(a2.basicSimplify(), NumScalar(d2)))

      case Add(scalar_expr, NumScalar(number)) =>
        if (number == 0) scalar_expr.basicSimplify() // Ignore Zero Numeric
        else Add(NumScalar(number), scalar_expr.basicSimplify()) // Change the Add Tree Order

      case Add(NumScalar(number), scalar_expr) =>
        if (number == 0) scalar_expr.basicSimplify() // Ignore Zero Numeric
        else Add(NumScalar(number), scalar_expr.basicSimplify())

      // Simplify Individual scalars
      case _ => Add(u.basicSimplify(), v.basicSimplify())
    }

  }
}

case class Dot(u: VectorExpr, v: VectorExpr) extends ScalarExpr {
  // u dot v infix
  override def basicSimplify(): ScalarExpr = {
    this match {
      case Dot(u, v) => Dot(u.basicSimplify(), v.basicSimplify())
      case _ => this
    }
  }

  override def is_member(element: Any): Boolean = {
    this match {
      case Dot(a, b) => a.is_member(element) || b.is_member(element)
      case _ => false
    }
  }

}

