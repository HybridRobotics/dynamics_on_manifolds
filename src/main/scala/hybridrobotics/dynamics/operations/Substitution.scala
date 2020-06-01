package hybridrobotics.dynamics.operations

object Substitution
{

  // switch out vector with its replacement.
  def vswap( v:VExp, old:VExp, sub:VExp ) : VExp = v match
  {
    case deltaV(u)   => if (old == deltaV(u))   {sub} else {deltaV(u)}
    case VAdd(u,v)  => if (old == VAdd(u,v))  {sub} else {VAdd(vswap(u, old, sub),vswap(v, old, sub))}
    case SMul(u,v)  => if (old == SMul(u,v))  {sub} else {SMul(vswap(u, old, sub),vsubstitute(v, old, sub))}
    case Cross(u,v) => if (old == Cross(u,v)) {sub} else {Cross(vswap(u, old, sub),vswap(v, old, sub))}
    case v:VExp     => if (old == v)          {sub} else {v}
  }

  // substitution
  def vsubstitute( e:Exp, old:VExp, sub:VExp ) : Exp = e match
  {

    case Par(u)   => Par(vsubstitute(u, old, sub))
    case Mul(u,v) => Mul(vsubstitute(u, old, sub),vsubstitute(v, old, sub))
    case Add(u,v) => Add(vsubstitute(u, old, sub),vsubstitute(v, old, sub))
    case Dot(u,v) => Dot(vswap(u, old, sub),vswap(v, old, sub))
    case u:Exp => u
  }

  def substitute( e:Exp, old:Exp, sub:Exp ) : Exp = e match
  {

    case Par(u)   => Par(substitute(u, old, sub))
    case Mul(u,v) => if (old == Mul(u,v)) {sub} else {Mul(substitute(u, old, sub),substitute(v, old, sub))}
    case Add(u,v) => if (old == Add(u,v)) {sub} else {Add(substitute(u, old, sub),substitute(v, old, sub))}
    case Dot(u,v) => if (old == Dot(u,v)) {sub} else {Dot(u,v)}
    case u:Exp  => if (old == u) {sub} else {u}
  }
}
