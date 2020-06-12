package hybridrobotics.dynamics.operations

object Differentiation {

  def diff(e: Any): Any = e match {
    case e: Exp => diffS(e)
    case e: VExp => diffV(e)
    case e: SO3 => e.getDiff
    case e: S2 => e.getDiff
    case e: MExp => diffM(e)
  }

  // differentiate expression
  def diffM(m: MExp): MExp = m match {
    case SMMul(u: MExp, v: Exp) => MAdd(SMMul(diffM(u), v), SMMul(u, diffS(v)))
    case MMul(u: MExp, v: MExp) => MAdd(MMul(diffM(u), v), MMul(u, diffM(v)))
    case MAdd(u: MExp, v: MExp) => MAdd(diffM(u), diffM(v))
    case CMat(s) => {
      SMMul(m, Num(0))
    }
    case CSMat(s) => {
      SMMul(m, Num(0))
    }
    case Mat(s) => Mat(s + "dot")
    case SMat(s) => Mat(s + "dot")
    case SkewMat(s) => Mat(s + "dot")
    case deltaM(m: MExp) => deltaM(diffM(m))
    case transpose(m: MExp) => transpose(diffM(m))
    case SO3(s) => Mat(s+"dot")
  }

  def diffV(v: VExp): VExp = v match {
    case Cross(a, b) => VAdd(Cross(diffV(a), b), Cross(a, diffV(b)))
    case VAdd(a, b) => VAdd(diffV(a), diffV(b))
    case SMul(a, b) => SMul(diffV(a), b) + SMul(a, diffS(b))
    case VMul(a, b) => VMul(diffM(a), b) + VMul(a, diffV(b))
    case Vec(s) => Vec(s + "dot")
    case UVec(s) => Vec(s + "dot")
    case CVec(s) => {
      SMul(v, Num(0))
    }
    case deltaV(v: VExp) => deltaV(diffV(v))
    case AVec(s, u) => AVec(s + "dot", diffV(u))
    case S2(s) => Vec(s+"dot")
  }

  def diffS(e: Exp): Exp = e match {
    case Num(d) => Num(0) // diff of constant zero
    case Var(s) => Var(s + "dot") // x becomes dx
    case Cons(s) => Num(0) // x becomes dx
    case deltaS(u) => deltaS(diffS(u))
    case Par(u) => Par(diffS(u))
    case Mul(u, v) => Mul(diffS(u), v) + Mul(u, diffS(v))
    case Add(u, v) => diffS(u) + diffS(v)
    case Dot(a, b) => Dot(diffV(a), b) + Dot(a, diffV(b))
  }
}
