package hybridrobotics.dynamics.operations

import hybridrobotics.dynamics.operations._
import TreeProperties._

object Expansion
{

  // expand expression
  def expandM( m:MExp ) : MExp = m match
  {
    case MAdd(u,v) => MAdd(expandM(u),expandM(v))
    case MMul(m,MAdd(u,v)) => MAdd(MMul(m,expandM(u)),MMul(m,expandM(v)))
    case MMul(MAdd(u,v),m) => MAdd(MMul(m,expandM(u)),MMul(m,expandM(v)))
    case MMul(m,u) =>
        if (nestedAddsM(MMul(m,u))) {expandM(MMul(expandM(m), expandM(u)))}
        else {MMul(m,u)}
    case SMMul(MAdd(a,b),v) => expandM(MAdd(SMMul(a,v),SMMul(b,v)))
    case SMMul(v,Add(a,b)) => expandM(MAdd(SMMul(v,a),SMMul(v,b)))
    case SMMul(u,v) => if (expandM(u) != u) {expandM(SMMul(expandM(u),v))} else if (expand(v) != v) {expandM(SMMul(u,expand(v)))} else {SMMul(u,v)}
    case m:MExp => m
  }


  def expandV( v:VExp ) : VExp = v match
  {
    case VAdd(u,v) => expandV(u) + expandV(v)
    case VMul(MAdd(u,v),w) => VAdd(expandV(VMul(u,w)),expandV(VMul(v,w)))
    case VMul(m,VAdd(u,v)) => VAdd(expandV(VMul(m,u)),expandV(VMul(m,v)))
    case VMul(m,u) => if (nestedAddsV(VMul(m,u))) {expandV(VMul(expandM(m), expandV(u)))} else {VMul(m,u)}

    case SMul(VAdd(a,b),v) => expandV(SMul(a,v)+SMul(b,v))
    case SMul(u,v) => if (nestedAddsV(SMul(u,v))) {expandV(SMul(expandV(u),expand(v)))} else {SMul(u,v)}

    case Cross(VAdd(a,b),VAdd(c,d)) => VAdd(expandV(Cross(a,c)),VAdd(expandV(Cross(a,d)),VAdd(expandV(Cross(b,c)),expandV(Cross(b,d)))))
    case Cross(VAdd(a,b),c) => VAdd(expandV(Cross(a,c)),expandV(Cross(b,c)))
    case Cross(c,VAdd(a,b)) => VAdd(expandV(Cross(a,c)),expandV(Cross(b,c)))
    case Cross(a,b) =>
        if (nestedAddsV(Cross(a,b))) {expandV(Cross(expandV(a),expandV(b)))} else {Cross(a,b)}


    case v:VExp => v
  }

  def expand ( e:Exp ) : Exp = e match
  {
    case Add(u,v) => Add(expand(u),expand(v))

    case Mul(Add(u,v),Add(w,x)) => Add(expand(Mul(u,w)),Add(expand(Mul(v,w)),Add(expand(Mul(u,x)),expand(Mul(v,x)))))
    case Mul(Add(u,v),w) => Add(expand(Mul(u,w)),expand(Mul(v,w)))
    case Mul(w,Add(u,v)) => Add(expand(Mul(u,w)),expand(Mul(v,w)))
    case Mul(u,v) =>
      if (nestedAdds(Mul(u,v))) {expand(Mul(expand(u),expand(v)))} else {Mul(u,v)}

    case Dot(VAdd(a,b),VAdd(c,d)) => Add(expand(Dot(a,c)),Add(expand(Dot(a,d)),Add(expand(Dot(b,c)),expand(Dot(b,d)))))
    case Dot(VAdd(a,b),c) => Add(expand(Dot(a,c)),expand(Dot(b,c)))
    case Dot(c,VAdd(a,b)) => Add(expand(Dot(a,c)),expand(Dot(b,c)))
    case Dot(a,b) =>
      if (nestedAdds(Dot(a,b))) {expand(Dot(expandV(a),expandV(b)))} else {Dot(a,b)}

    case u:Exp => u
  }

  // expands complete abstract syntax tree
  def expansion ( e:Exp ) : Exp =
  {
    var A = expand(e)
    return A
  }
}
