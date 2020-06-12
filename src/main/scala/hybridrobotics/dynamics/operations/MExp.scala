// MExp.scala
package hybridrobotics.dynamics.operations

class MExp {
  //Wrap s:String to MExp

  import language.implicitConversions

  implicit def str2MExp(s: String, t: String): MExp = Mat(s)

  //Infix operators
  def *(u: Exp): MExp = SMMul(this, u)

  def **(v: VExp): VExp = VMul(this, v)

  def ***(m: MExp): MExp = MMul(this, m)

  def +(m: MExp): MExp = MAdd(this, m)

  def -(m: MExp): MExp = MAdd(this, SMMul(m, Num(-1)))

  def detm: Exp = det(this)

  def T: MExp = transpose(this)
}

// Set of all vector expression classes
case class SMMul(u: MExp, v: Exp) extends MExp // u *   v infix
case class VMul(u: MExp, v: VExp) extends VExp // u **  v infix
case class MMul(u: MExp, v: MExp) extends MExp // u *** v infix
case class MAdd(u: MExp, v: MExp) extends MExp // u +   v infix
case class det(u: MExp) extends Exp // det prefix
case class deltaM(m: MExp) extends MExp // delta prefix
case class transpose(m: MExp) extends MExp // transpose prefix
case class Mat(s: String) extends MExp
case class SMat(s: String) extends MExp with SymmetricMatrix
case class CSMat(s: String) extends MExp with ConstantMatrix with SymmetricMatrix
case class CMat(s: String) extends MExp with ConstantMatrix
case class SkewMat(s: String) extends MExp with SkewSymmetricMatrix


// SO3
case class SO3(s: String) extends MExp with SpecialEuclidean {
  val etaStr: String = "eta_{" + s + "}"
  val OmegaStr: String = "Omega_{" + s + "}"

  override def detm: Exp = Num(1.0) // determinant of SO3 is 1

  def getEta: Vec = Vec(this.etaStr)
  def getDelta: MExp = MMul(SO3(s), SkewMat(this.etaStr))
  def getDiff: MExp = MMul(SO3(s), SkewMat(this.OmegaStr))
}

//case class deltaM(m: SO3) extends MExp  {
//
//}



