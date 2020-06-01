package hybridrobotics.dynamics.operations

import hybridrobotics.dynamics.operations.Substitution._
import hybridrobotics.dynamics.operations.TreeProperties._

object Collection
{


  // Collect scalar expression with respect to scalar z.
  def colScalar( e:Exp, z:Exp ) : Exp = e match
  {

      // ADDITION
      case Add(u,v) => colScalar(u,z) + colScalar(v,z)

      case Mul(u,v) =>
        if (isMember(Mul(u,v),z)) {substitute(Mul(u,v),z,Num(1.0))}
        else {Mul(u,v)}

      case u:Exp => u
  }




  // Collect scalar expression with respect to vector z.
  def col( e:Exp, z:VExp ) : Exp = e match
  {
    // Base Cases

    // ADDITION
    case Add(u,v) => col(u,z) + col(v,z)

    // MULTIPLICATION
    case Mul(u,v) => col(u,z) * col(v,z)


    // Potential Energy
    case Dot(u,VMul(MMul(MMul(r,SkewMat(s)),y),v)) =>
        if (Vec(s) == z) {Dot(Vec(s), Cross(VMul(transpose(r)***y,u),v))}
        else {Dot(u,VMul(MMul(MMul(r,SkewMat(s)),y),v))}
    case Dot(VMul(MMul(r,SkewMat(s)),v),u) =>
        if (Vec(s) == z) {Dot(Vec(s), Cross(VMul(transpose(r),u),v))}
        else {Dot(VMul(MMul(r,SkewMat(s)),v),u)}

    // Kinetic Energy Terms
    case Dot(Cross(b,c),VMul(u,v)) => col(Dot(VMul(u,v),Cross(b,c)),z)
    case Dot(VMul(u,v),Cross(b,c)) =>
        if (b == z) {Dot(b,Cross(c,VMul(u,v)))}
        else if (c == z) {Dot(c,Cross(VMul(u,v),b))}
        else if (v == z) {Dot(v,Cross(VMul(u,b),c))}
        else {Dot(VMul(u,v),Cross(b,c))}

    case Dot(VMul(u,v),a)  => if (v == z) {Dot(v,VMul(u,a))} else {Dot(a,VMul(u,v))}
    case Dot(a,VMul(u,v)) => col(Dot(VMul(u,v),a),z)

    case Dot(a,Cross(b,c)) =>
        if (b == z) {Dot(b,Cross(c,a))}
        else if (c == z) {Dot(c,Cross(a,b))}
        else {Dot(a,Cross(b,c))}
    case Dot(Cross(b,c),a) => col(Dot(a,Cross(b,c)),z)

    case Dot(a,b) => if (b == z) {Dot(b,a)} else Dot(a,b)

    case u:Exp => u
  }


}
