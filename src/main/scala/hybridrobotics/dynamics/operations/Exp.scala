// Exp.scala
package hybridrobotics.dynamics.operations

class Exp
{
  // Wrap i:Int and d:Double to Num(d) & String to Var(s)
  import language.implicitConversions
  implicit def int2Exp( i:Int    ) : Exp = Num(i.toDouble)
  implicit def dbl2Exp( d:Double ) : Exp = Num(d)
  implicit def str2Exp( s:String ) : Exp = Var(s)

  // Infix operators from high to low using Scala precedence
  def *   ( v:Exp ) : Exp = Mul(this,v)
  def +   ( v:Exp ) : Exp = Add(this,v)
  def -   ( v:Exp ) : Exp = Add(this,Mul(Num(-1),v))   // Subtract is stored as adding a negative
}

// Expression Case Classes
case class deltaS( u:Exp )     extends Exp  // delta prefix
case class Num( n:Double )     extends Exp // wrap double
case class Var( s:String )     extends Exp // wrap String
case class Cons( s:String )    extends Exp // wrap String (Constant)
case class Par( u:Exp )        extends Exp // parentheses
case class Mul( u:Exp, v:Exp ) extends Exp // u * v infix
case class Add( u:Exp, v:Exp ) extends Exp // u + v infix
