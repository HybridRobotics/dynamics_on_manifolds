package hybridrobotics.dynamics.operations

object TreeProperties {

  // is node element a literal
  def isLeaf(e: Exp): Boolean = e match {
    case Add(u, v) => return false
    case Mul(u, v) => return false
    case Dot(a, b) => return false
    case u: Exp => return true
  }

  // is a scalar element z in expression e
  def isMember(e: Exp, z: Exp): Boolean = e match {
    case Add(u, v) => isMember(u, z) && isMember(v, z)
    case Mul(u, v) => isMember(u, z) && isMember(v, z)
    case u: Exp => if (u == z) {
      return true
    } else {
      return false
    }
  }

  // find violations of the expanded structure
  def nestedAdds(e: Exp): Boolean = e match {
    case Add(u, v) => return true
    case Mul(u, v) => return nestedAdds(u) || nestedAdds(v)
    case Dot(a, b) => return nestedAddsV(a) || nestedAddsV(b)
    case u: Exp => return false
  }

  def nestedAddsV(v: VExp): Boolean = v match {
    case VAdd(u, v) => return true
    case VMul(m, v) => return nestedAddsM(m) || nestedAddsV(v)
    case SMul(v, u) => return nestedAdds(u) || nestedAddsV(v)
    case Cross(a, b) => return nestedAddsV(a) || nestedAddsV(b)
    case v: VExp => return false
  }


  def nestedAddsM(m: MExp): Boolean = m match {
    case MAdd(u, v) => return true
    case MMul(m, p) => return nestedAddsM(m) || nestedAddsM(p)
    case SMMul(m, u) => return nestedAdds(u) || nestedAddsM(m)
    case m: MExp => return false
  }


  // nested scalars searched to combine constant coefficiencts
  def nestedScalars(e: Exp): Boolean = e match {
    case Add(u, v) => return nestedScalars(u) || nestedScalars(v)
    case Mul(u, v) => return nestedScalars(u) || nestedScalars(v)
    case Dot(a, b) => return nestedScalarsV(a) || nestedScalarsV(b)
    case u: Exp => return false
  }

  def nestedScalarsV(v: VExp): Boolean = v match {
    case VAdd(u, v) => return nestedScalarsV(u) || nestedScalarsV(v)
    case VMul(m, v) => return nestedScalarsM(m) || nestedScalarsV(v)
    case SMul(v, u) => return true
    case Cross(a, b) => return nestedAddsV(a) || nestedAddsV(b)
    case v: VExp => return false
  }

  def nestedScalarsM(m: MExp): Boolean = m match {
    case MAdd(m, p) => return nestedScalarsM(m) || nestedScalarsM(p)
    case MMul(m, p) => return nestedScalarsM(m) || nestedScalarsM(p)
    case SMMul(m, u) => return true
    case m: MExp => return false
  }
}
