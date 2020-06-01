package hybridrobotics.dynamics.operations

object Separation
{

  // processes elements for separation and extraction of dynamics
  def splitter( e:Exp ) : (Exp, Exp) = e match
  {
    case Add(Add(u,v),w)   => splitter(Add(u,Add(v,w)))
    case Add(u,v)   => (u,v)
    case Mul(u,v)   => (Mul(u,v), Num(0))
    case Dot(a,b)   => (Dot(a,b),Num(0))
  }

  // returns two multiplied pieces of expression
  def numMultiple( e:Exp ) : (Exp, Exp) = e match
  {
    case Mul(Num(d),u) => (Num(d),u)
    case Mul(u,v) => (Num(1),Mul(u,v))
    case Dot(u,v) => (Num(1),Dot(u,v))
  }


  // returns two added pieces of expression.
  def extract( e:Exp, z:Exp ) : Exp = e match
  {
    case Add(u,v) => extract(u,z) + extract(v,z)
    case Mul(u,v) => if (u == z) {v}
    else if (v == z) {u}
    else extract(u,z) * extract(v,z)
        case u:Exp => u
  }


  // Returns two added pieces of expression.
  def extractAll(e:Exp) : VExp = e match
  {
    case Add(u,v) => extractAll(u) + extractAll(v)
    case Mul(u,v) => SMul(extractV2(Mul(u,v)),extractV1(Mul(u,v)))
    case u:Exp => Vec("0")
  }


  def extractV1( e:Exp) : Exp = e match
  {
    case Mul(Dot(a,b),c) => c
    case Mul(c,Dot(a,b)) => c
    case Mul(u,v) => extractV1(u) * extractV1(v)
    case u:Exp => u
  }

  def extractV2( e:Exp ) : VExp = e match
  {
    case Mul(Dot(a,b),c) => b
    case Mul(c,Dot(a,b)) => b
    case Mul(u,v) => if (extractV2(u) == Vec("None"))  {extractV2(v)} else {extractV2(u)}
    case u:Exp => Vec("None")
  }



  // returns two added pieces of expression.
  def separate( e:Exp, z:Exp ) : Exp = e match
  {
    case Add(u,Add(v,w)) =>
        if (collectable(u,z)) {u + separate(Add(v,w),z)}
        else {separate(Add(v,w),z)}
    case Add(u,v)   =>
        if (collectable(u,z) && collectable(v,z)) {Add(u,v)}
        else if (collectable(u,z)) {u}
        else if (collectable(v,z)) {v}
        else {Num(0)}
  }

  def separateV( e:Exp, z:VExp ) : Exp = e match
  {
    case Add(u,Add(v,w)) =>
      if (collectableV(u,z)) {u + separateV(Add(v,w),z)}
      else {separateV(Add(v,w),z)}
    case Add(u,v)   =>
        if (collectableV(u,z) && collectableV(v,z)) {Add(u,v)}
        else if (collectableV(u,z)) {u}
        else if (collectableV(v,z)) {v}
        else {Num(0)}
    case u:Exp => if (collectableV(u,z)) {u} else {Num(0)}
  }

  // finds vector to be collected for
  def collectable(e:Exp, z:Exp) : Boolean = e match
  {
    case Mul(u,v) => collectable(u,z) || collectable(v,z)
    case Dot(a,b) => false
    case u:Exp => if (u == z) {true} else {false}
  }

  def collectableV(e:Exp, z:VExp) : Boolean = e match
  {
    case Mul(u,v) => collectableV(u,z) || collectableV(v,z)
    case Dot(a,b) => if (a == z) {true} else {false}
    case u:Exp => false
  }

  // keep right side of an expression
  def rightSideV( e:Exp ) : VExp = e match
  {
    case Dot(a,b) => b
	case Mul(a,b) => Vec("zero") //bug in collect
  }

  def rightSide( e:Exp ) : Exp = e match
  {
    case Mul(a,b) => b
  }
}
