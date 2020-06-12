package hybridrobotics.dynamics.operations

object Variation {

  def delta (e:Any) : Any = e match {
    case e: Exp => deltaS(e)
    case e: VExp => deltaV(e)
    case e: SO3 => e.getDelta
    case e: MExp => deltaM(e)
  }

  // take variation of an expression
  def variationM(m: MExp): MExp = m match {
    case SMMul(u, v) => SMMul(variationM(u), v) + SMMul(u, variation(v))
    case MMul(u, v) => MMul(variationM(u), v) + MMul(u, variationM(v))
    case MAdd(u, v) => variationM(u) + variationM(v)
    case u: MExp =>
      if (u.isInstanceOf[ConstantMatrix]) {
        SMMul(u, Num(0))
      }
      else {
        deltaM(u)
      }
  }

  def variationV(v: VExp): VExp = v match {
    case VAdd(u, v) => variationV(u) + variationV(v)
    case VMul(a, b) =>
      if (a.isInstanceOf[ConstantMatrix]) {
        VMul(a, variationV(b))
      }
      else {
        VMul(variationM(a), b) + VMul(a, variationV(b))
      }
    case SMul(a, b) =>
      if (b.isInstanceOf[ConstantMatrix]) {
        SMul(variationV(a), b)
      }
      else {
        SMul(variationV(a), b) + SMul(a, variation(b))
      }
    case Cross(a, b) => Cross(variationV(a), b) + Cross(a, variationV(b))
    case v: VExp => if (v.isInstanceOf[ConstantVector]) {
      SMul(v, Num(0))
    } else {
      deltaV(v)
    }
  }

  def variation(e: Exp): Exp = e match {
    case Var(s) => deltaS(Var(s))
    case Cons(s) => Num(0)
    case Num(d) => Num(0)
    case Add(u, v) => Add(variation(u), variation(v))
    case Mul(u, v) => Mul(variation(u), v) + Mul(u, variation(v))
    case Dot(a, VMul(m, b)) =>
      if ((m.isInstanceOf[SymmetricMatrix]) && (a == b)) {
        Mul(Num(2), Dot(variationV(a), VMul(m, b)))
      }
      else {
        Dot(variationV(a), VMul(m, b)) + Dot(a, variationV(VMul(m, b)))
      }
    case Dot(a, b) => Dot(variationV(a), b) + Dot(a, variationV(b))
  }


  // take variation with respect to a desired trajectory
  def variationM_d(m: MExp): MExp = m match {
    case SMMul(u, v) => SMMul(variationM(u), desired(v)) + SMMul(desiredM(u), variation_d(v))
    case MMul(u, v) => MMul(variationM_d(u), desiredM(v)) + MMul(desiredM(u), variationM_d(v))
    case MAdd(u, v) => variationM_d(u) + variationM_d(v)
    case u: MExp =>
      if (u.isInstanceOf[ConstantMatrix]) {
        SMMul(desiredM(u), Num(0))
      }
      else {
        desiredM(u)
      }
  }

  def variationV_d(v: VExp): VExp = v match {
    case VAdd(u, v) => variationV_d(u) + variationV_d(v)
    case VMul(a, b) =>
      if (a.isInstanceOf[ConstantMatrix]) {
        VMul(desiredM(a), variationV(b))
      }
      else {
        VMul(variationM(a), desiredV(b)) + VMul(desiredM(a), variationV(b))
      }
    case SMul(a, b) =>
      if (b.isInstanceOf[ConstantMatrix]) {
        SMul(variationV_d(a), desired(b))
      }
      else {
        SMul(variationV_d(a), desired(b)) + SMul(desiredV(a), variation_d(b))
      }
    case Cross(a, b) => Cross(variationV_d(a), desiredV(b)) + Cross(desiredV(a), variationV_d(b))
    case v: VExp => if (v.isInstanceOf[ConstantVector]) {
      SMul(v, Num(0))
    } else {
      variationV(v)
    }
  }

  def variation_d(e: Exp): Exp = e match {
    case Var(s) => deltaS(Var(s))
    case Cons(s) => Num(0)
    case Num(d) => Num(0)
    case Add(u, v) => Add(variation_d(u), variation_d(v))
    case Mul(u, v) => Mul(variation_d(u), desired(v)) + Mul(desired(u), variation_d(v))
    case Dot(a, VMul(m, b)) =>
      if ((m.isInstanceOf[SymmetricMatrix]) && (a == b)) {
        Mul(Num(2), Dot(variationV_d(a), desiredV(VMul(m, b))))
      }
      else {
        Dot(variationV_d(a), desiredV(VMul(m, b))) + Dot(desiredV(a), variationV_d(VMul(m, b)))
      }
    case Dot(a, b) => Dot(variationV_d(a), desiredV(b)) + Dot(desiredV(a), variationV_d(b))
  }

  // compute desired trajectory from an expression
  def desiredM(m: MExp): MExp = m match {
    case SMMul(u: MExp, v: Exp) => SMMul(desiredM(u), desired(v))
    case MMul(u: MExp, v: MExp) => MMul(desiredM(u), desiredM(v))
    case MAdd(u: MExp, v: MExp) => MAdd(desiredM(u), desiredM(v))
    case transpose(u: MExp) => transpose(desiredM(u))
    case Mat(s: String) => Mat(s + "_d")
    case SMat(s: String) => SMat(s + "_d")
    case CSMat(s: String) => CSMat(s)
    case CMat(s: String) => CMat(s)
    case SkewMat(s: String) => SkewMat(s + "_d")
  }

  def desiredV(v: VExp): VExp = v match {
    case Cross(u: VExp, v: VExp) => Cross(desiredV(u), desiredV(v))
    case SMul(u: VExp, v: Exp) => SMul(desiredV(u), desired(v))
    case VAdd(u: VExp, v: VExp) => VAdd(desiredV(u), desiredV(v))
    case Vec(s: String) => Vec(s + "_d")
    case UVec(s: String) => UVec(s + "_d")
    case CVec(s: String) => CVec(s)
    case AVec(s: String, u: VExp) => AVec(s + "_d", desiredV(u))
    case VMul(u: MExp, v: VExp) => VMul(desiredM(u), desiredV(v))
  }

  def desired(e: Exp): Exp = e match {
    case Num(n) => Num(n)
    case Var(s) => Var(s + "_d")
    case Cons(s) => Cons(s)
    case Mul(u, v) => Mul(desired(u), desired(v))
    case Add(u, v) => Add(desired(u), desired(v))
    case Dot(u, v) => Dot(desiredV(u), desiredV(v))
    //	case det(u,v) => det(desiredM(u),desiredM(v))
  }


}
