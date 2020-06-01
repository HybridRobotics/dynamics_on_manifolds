package hybridrobotics.dynamics.operations

import hybridrobotics.dynamics.operations._
import Collection._
import Differentiation._
import Expansion._
import Simplification._
import PrintLine._

object IntegrationByParts
{

  // integration removes a "dot" from vectors string identifier
  def integrate( v:VExp ) : VExp = v match
  {
    case Vec(s) => Vec(s.replaceFirst("dot",""))
    case deltaV(Vec(s)) => deltaV(Vec(s.replaceFirst("dot","")))
  }

  def integrateScalar( e:Exp ) : Exp = e match
  {
    case Var(s) => Var(s.replaceFirst("dot",""))
    case deltaS(Var(s)) => deltaS(Var(s.replaceFirst("dot","")))
  }


  // implements integration by parts on a collection term
  def ibp( e:Exp, old:VExp) : Exp = e match
  {

    case Par(u)   => Par(ibp(u, old))
    case Mul(u,v) => {Mul(ibp(u, old),ibp(v, old))}
    case Add(u,v) => {Add(ibp(u, old),ibp(v, old))}
    case Dot(u,v) => if (old == u) {Dot(integrate(u)*Num(-1),diffV(v))} else {Dot(u,v)}
    case u:Exp => u

  }

  // perform integration by parts
  def ibpScalar( e:Exp, old:Exp) : Exp = e match
  {
    case Add(u,v) => ibpScalar(u,old) + ibpScalar(v,old)
    case Mul(u,v) => if (u == old) {Mul(Num(-1),integrateScalar(u))*diffS(v)} else {u*v}
    case u:Exp => u

  }

  def integrateByPartsV( e:Exp, v:List[VExp]) : Exp =
  {
    var a = expansion(e)
    a = simplify(a)
    for  (vector <- v)
    {
      a = col(a, vector)
      a = ibp(a, vector)
      a = simplify(a)
    }

    return a
  }

  // spans generalized vectors to perform integration by parts on their derivatives
  def integrateByParts( e:Exp, s:List[Exp]) : Exp =
  {
    var a = expansion(e)
    a = simplify(a)
    for  (scalar <- s)
    {
      a = colScalar(a, scalar)
      a = ibpScalar(a, scalar)
      a = simplify(a)
    }

    return a
  }

}
