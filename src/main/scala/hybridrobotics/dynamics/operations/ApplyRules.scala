package hybridrobotics.dynamics.operations

object ApplyRules {
  // Substitute for kinematic constraints and variations on configuration manifolds
  def applyConstraintsM(m: MExp, mrules: Map[MExp, MExp]): MExp = m match {
    case MAdd(u, v) => MAdd(applyConstraintsM(u, mrules), applyConstraintsM(v, mrules))
    case MMul(u, v) => MMul(applyConstraintsM(u, mrules), applyConstraintsM(v, mrules))
    case u: MExp => if (mrules.contains(m)) mrules(m) else u

  }

  def applyConstraintsV(v: VExp, vrules: Map[VExp, VExp], mrules: Map[MExp, MExp]): VExp = v match {
    case deltaV(u) => if (vrules.contains(deltaV(u))) {
      vrules(deltaV(u))
    } else {
      deltaV(u)
    }
    case SMul(u, v) => SMul(applyConstraintsV(u, vrules, mrules), v)
    case VAdd(u, v) =>
      if (vrules.contains(VAdd(u, v))) {
        vrules(VAdd(u, v))
      }
      else {
        VAdd(applyConstraintsV(u, vrules, mrules), applyConstraintsV(v, vrules, mrules))
      }
    case Cross(u, v) =>
      if (vrules.contains(Cross(u, v))) {
        vrules(Cross(u, v))
      }
      else {
        Cross(applyConstraintsV(u, vrules, mrules), applyConstraintsV(v, vrules, mrules))
      }
    case VMul(a, b) => VMul(applyConstraintsM(a, mrules), applyConstraintsV(b, vrules, mrules))
    case v: VExp => if (vrules.contains(v)) {
      vrules(v)
    } else {
      v
    }
  }

  def applyConstraints(e: Exp, rules: Map[Exp, Exp], vrules: Map[VExp, VExp], mrules: Map[MExp, MExp]): Exp = e match {
    case Par(u) => Par(applyConstraints(u, rules, vrules, mrules))
    case Mul(u, v) => Mul(applyConstraints(u, rules, vrules, mrules), applyConstraints(v, rules, vrules, mrules))
    case Add(u, v) => Add(applyConstraints(u, rules, vrules, mrules), applyConstraints(v, rules, vrules, mrules))
    case Dot(u, v) =>
      if (rules.contains(Dot(u, v))) {
        applyConstraints(rules(Dot(u, v)), rules, vrules, mrules)
      }
      else if (rules.contains(Dot(v, u))) {
        applyConstraints(rules(Dot(v, u)), rules, vrules, mrules)
      }
      else {
        Dot(applyConstraintsV(u, vrules, mrules), applyConstraintsV(v, vrules, mrules))
      }
    case u: Exp => u
  }
}
