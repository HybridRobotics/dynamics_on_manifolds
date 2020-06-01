// VExp.scala
package hybridrobotics.dynamics.operations

class VExp
{
  //Wrap s:String to VExp
  import language.implicitConversions
  implicit def str2VExp(s:String) : VExp = Vec(s)

  //Infix operators
  def x ( v:VExp ) : VExp = Cross(this,v)
  def * ( v:Exp  ) : VExp = SMul(this,v)
  def + ( v:VExp ) : VExp = VAdd(this,v)
  def - ( v:VExp ) : VExp = VAdd(this,SMul(v,Num(-1)))
  def dot    ( v:VExp ) :  Exp  = Dot(this,v)
  def deltaV ( v:VExp ) : VExp  = deltaV(this)
}

// Set of all vector expression classes
case class Cross( u:VExp, v:VExp  ) extends VExp  // u x v infix
case class SMul(  u:VExp, v:Exp   ) extends VExp  // u * v infix
case class VAdd(  u:VExp, v:VExp  ) extends VExp  // u + v infix
case class Dot(   u:VExp, v:VExp  ) extends Exp   // u dot v infix
case class deltaV( u:VExp         ) extends VExp  // delta prefix
case class Vec( s:String) extends VExp            // string wraped to VExp
case class UVec(s:String) extends VExp with UnitVector     // string wraped to VExp
case class CVec(s:String) extends VExp with ConstantVector // string wraped to VExp
case class AVec(s:String,u:VExp) extends VExp // holds symbolic reference for large vector
case class ZVec(s:String) extends VExp // zero vector


