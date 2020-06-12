package hybridrobotics.dynamics.operations

import TreeProperties._
import PrintLine._
import Expansion._

object Simplification {

  // Organize expression with respect to subclass type, then alphabetically.
  def org(e: Exp): Exp = e match {

    case Add(u, v) => org(u) + org(v)
    case Dot(Cross(a, b), c) => org(Dot(c, Cross(a, b)))

    case Dot(a, VMul(u, Cross(b, c))) => org(Dot(VMul(u, a), Cross(b, c)))
    case Dot(VMul(u, Cross(b, c)), a) => org(Dot(VMul(u, a), Cross(b, c)))

    case Dot(VMul(u, Vec(s1)), Cross(Vec(s2), Vec(s3))) =>
      if ((s1 < s2) && (s2 < s3)) {
        Dot(Vec(s1), Cross(VMul(u, Vec(s2)), Vec(s3)))
      }
      else if ((s1 < s3) && (s3 < s2)) {
        Dot(Vec(s1), Cross(VMul(u, Vec(s2)), Vec(s3)))
      }
      else if ((s2 < s1) && (s1 < s3)) {
        Dot(Vec(s2), Cross(VMul(u, Vec(s3)), Vec(s1)))
      }
      else if ((s2 < s3) && (s3 < s1)) {
        Dot(Vec(s2), Cross(VMul(u, Vec(s3)), Vec(s1)))
      }
      else if ((s3 < s1) && (s1 < s2)) {
        Dot(Vec(s3), Cross(VMul(u, Vec(s1)), Vec(s2)))
      }
      else {
        Dot(Vec(s3), Cross(VMul(u, Vec(s1)), Vec(s2)))
      }

    case Dot(Vec(s1), Vec(s2)) => if (s1 < s2) {
      Dot(Vec(s1), Vec(s2))
    } else {
      Dot(Vec(s2), Vec(s1))
    }
    case Dot(Vec(s1), Cross(Vec(s2), Vec(s3))) =>
      if ((s1 < s2) && (s2 < s3)) {
        Dot(Vec(s1), Cross(Vec(s2), Vec(s3)))
      }
      else if ((s1 < s3) && (s3 < s2)) {
        Dot(Vec(s1), Cross(Vec(s2), Vec(s3)))
      }
      else if ((s2 < s1) && (s1 < s3)) {
        Dot(Vec(s2), Cross(Vec(s3), Vec(s1)))
      }
      else if ((s2 < s3) && (s3 < s1)) {
        Dot(Vec(s2), Cross(Vec(s3), Vec(s1)))
      }
      else if ((s3 < s1) && (s1 < s2)) {
        Dot(Vec(s3), Cross(Vec(s1), Vec(s2)))
      }
      else {
        Dot(Vec(s3), Cross(Vec(s1), Vec(s2)))
      }
    case Dot(a, b) => Dot(a, b)

    case Mul(Num(d), u) => Mul(Num(d), org(u))
    case Mul(Dot(a, b), u) => org(Mul(u, Dot(a, b)))
    case Mul(Mul(u, v), Dot(a, b)) => org(Mul(u, Mul(v, Dot(a, b))))
    case Mul(u, Dot(a, b)) =>
      if (org(u) != u) org(Mul(org(u), org(Dot(a, b))))
      else {
        Mul(u, org(Dot(a, b)))
      }
    case Mul(Mul(u, Dot(a, b)), v) => org(Mul(u, Mul(v, Dot(a, b))))

    case Mul(Var(s), Var(t)) =>
      if (s <= t) {
        Mul(Var(s), Var(t))
      }
      else {
        Mul(Var(t), Var(s))
      }

    case Mul(Var(s), Mul(Var(t), u)) =>
      if (org(Mul(Var(t), u)) != Mul(Var(t), u)) {
        org(Mul(Var(s), org(Mul(Var(t), u))))
      }
      else if (s <= t) {
        Mul(Var(s), Mul(Var(t), u))
      }
      else {
        org(Mul(Var(t), Mul(Var(s), u)))
      }

    case Mul(u, Var(s)) => org(Mul(Var(s), u))

    case Mul(Mul(Var(s), u), Mul(Var(t), v)) =>
      if (s <= t) {
        org(Mul(Var(s), Mul(Var(t), Mul(u, v))))
      }
      else {
        org(Mul(Var(t), Mul(Var(s), Mul(u, v))))
      }

    case Mul(u, v) => Mul(org(u), org(v))
    case u: Exp => u
  }

  // Gets rid of extraneous nodes. (Mulitiplication by 1)
  def removeOneCoefsV(v: VExp): VExp = v match {
    case VAdd(u, v) => removeOneCoefsV(u) + removeOneCoefsV(v)
    case SMul(a, b) =>
      if (b == Num(1)) {
        removeOneCoefsV(a)
      }
      else if ((removeOneCoefsV(a) != a) || ((removeOneCoefs(b) != b))) removeOneCoefsV(SMul(removeOneCoefsV(a), removeOneCoefs(b)))
      else SMul(a, b)
    case Cross(u, v) => Cross(removeOneCoefsV(u), removeOneCoefsV(v))
    case VMul(a, b) => VMul(a, b)
    case v: VExp => v

  }

  def removeOneCoefs(e: Exp): Exp = e match {

    case Add(u, v) => Add(removeOneCoefs(u), removeOneCoefs(v))
    case Mul(u, v) =>
      if (u == Num(1)) {
        removeOneCoefs(v)
      }
      else if (v == Num(1)) {
        removeOneCoefs(u)
      }
      else if ((removeOneCoefs(u) != u) || ((removeOneCoefs(v) != v))) {
        removeOneCoefs(Mul(removeOneCoefs(u), removeOneCoefs(v)))
      }
      else {
        Mul(removeOneCoefs(u), removeOneCoefs(v))
      }
    case Dot(a, b) => Dot(removeOneCoefsV(a), removeOneCoefsV(b))
    case u: Exp => u
  }

  // combine numberic constants
  def combine(e: Exp): Double = e match {
    case Mul(u, v) => combine(u) * combine(v);
    case Num(d) => return d
    case u: Exp => return 1
  }

  // determines if remaining elements in tree are all numbers
  def terms(e: Exp): Exp = e match {
    case Mul(Num(d), v) => terms(v)
    case Mul(v, Num(d)) => terms(v)
    case Mul(u, v) =>
      if (anyTerms(u) && anyTerms(v)) {
        terms(u) * terms(v)
      }
      else if (anyTerms(u)) {
        terms(u)
      }
      else {
        terms(v)
      }
    case v: Exp => v
  }

  def anyTerms(e: Exp): Boolean = e match {
    case Mul(u, v) => anyTerms(u) || anyTerms(v)
    case Num(d) => false
    case u: Exp => true
  }

  // find zero terms for elimination
  def findZeros(e: Exp): Boolean = e match {
    case Mul(u, v) => findZeros(u) || findZeros(v)
    case Num(d) => if (d == 0) {
      return true
    } else {
      return false
    }
    case u: Exp => false
  }


  // simplifies expression, includes combining constants and eliminating zero terms
  // also checks vector properties
  // tree must be in expanded form
  def simp(e: Exp): Exp = e match {
    // Will remove zeros.
    case Add(Num(d), Num(e)) => Num(d + e)
    case Add(u, Num(d)) => simp(Add(Num(d), u))
    case Add(Num(d), u) => if (d == 0) {
      simp(u)
    } else {
      Add(Num(d), simp(u))
    }
    case Add(u, v) => Add(simp(u), simp(v))

    // Gather numeric constants in various term structures.
    // In the expression Num(d*e), d*e relies on scala's built in math package.
    case Mul(u, v) => if (findZeros(Mul(u, v))) {
      Num(0)
    } else {
      Mul(Num(combine(Mul(u, v))), terms(Mul(u, v)))
    }
    case Dot(a, b) => Dot(a, b)
    case u: Exp => u

  }

  def simpV(e: VExp): VExp = e match {
    // Will remove zeros.
    case VAdd(vector1, SMul(vector2, Num(scalar1))) => if (scalar1 == 0) {
      simpV(vector1)
    } else {
      VAdd(simpV(vector1), SMul(simpV(vector2), simp(Num(scalar1))))
    }
    case SMul(vector1, scalar1) => SMul(simpV(vector1), simp(scalar1))
    case VAdd(vector1, vector2) => VAdd(simpV(vector1), simpV(vector2))
    case vector: VExp => vector

    //    // Gather numeric constants in various term structures.
    //    // In the expression Num(d*e), d*e relies on scala's built in math package.
    //    case Mul(u, v) => if (findZeros(Mul(u, v))) {
    //      Num(0)
    //    } else {
    //      Mul(Num(combine(Mul(u, v))), terms(Mul(u, v)))
    //    }
    //    case Dot(a, b) => Dot(a, b)
    //    case u: Exp => u

  }


  def vectorRules(e: Exp): Exp = e match {
    case Add(u, v) => vectorRules(u) + vectorRules(v)
    case Mul(u, v) => vectorRules(u) * vectorRules(v)

    // Gather numeric constants nested in vector expressions (scalar multiples)
    case Dot(Cross(b, c), a) => simp(Dot(a, Cross(b, c)))
    case Dot(a, Cross(b, c)) => if ((a == b) || (a == c) || (b == c)) {
      Num(0)
    } else {
      Dot(a, Cross(b, c))
    }
    case Dot(a: VExp, b: VExp) =>
      if ((a.isInstanceOf[UnitVector]) && (b.isInstanceOf[UnitVector]) && (a == b)) {
        Num(1)
      }
      else {
        Dot(a, b)
      }

    case u: Exp => u
  }

  // removes the zero terms from the tree.
  def removeZeros(e: Exp): Exp = e match {
    case Add(Num(d), v) => if (d == 0) {
      removeZeros(v)
    } else {
      Add(Num(d), removeZeros(v))
    }
    case Add(u, Num(d)) => if (d == 0) {
      removeZeros(u)
    } else Add(removeZeros(u), Num(d))
    case Add(u, v) => {
      Add(removeZeros(u), removeZeros(v))
    }
    case Mul(u, v) => Mul(u, v)
    case Dot(a, b) => Dot(a, b)
    case Num(d) => Num(d)
    case Var(s) => Var(s)
    case Cons(s) => Cons(s)
  }


  // pull scalar components out of vector and matrix expressions
  def pullM(m: MExp): MExp = m match {
    case MAdd(u, v) => pullM(u) + pullM(v)

    case MMul(m, SMMul(u, v)) => SMMul(MMul(m, u), v)
    case MMul(SMMul(u, v), m) => SMMul(MMul(m, u), v)
    case MMul(m, u) =>
      if (nestedScalarsM(MMul(m, u))) {
        pullM(MMul(pullM(m), pullM(u)))
      }
      else {
        MMul(m, u)
      }

    case SMMul(SMMul(a, b), v) => pullM(SMMul(a, b * v))
    case SMMul(u, v) => if (nestedScalarsM(u)) {
      pullM(SMMul(pullM(u), v))
    } else {
      SMMul(u, v)
    }
    case m: MExp => m
  }

  def pullV(v: VExp): VExp = v match {

    case VMul(SMMul(u, v), w) => pullV(SMul(VMul(u, w), v))
    case VMul(m, SMul(u, v)) => pullV(SMul(VMul(m, u), v))
    case VMul(u, v) => if (nestedScalarsV(VMul(u, v))) {
      pullV(VMul(pullM(u), pullV(v)))
    } else {
      VMul(u, v)
    }

    case SMul(SMul(u, v), w) => pullV(SMul(u, v * w))
    case SMul(u, v) => if (nestedScalarsV(u)) {
      pullV(SMul(pullV(u), v))
    } else SMul(u, v)

    case Cross(SMul(u, v), SMul(w, x)) => pullV(SMul(u x w, v * x))
    case Cross(a, SMul(u, v)) => pullV(SMul(a x u, v))
    case Cross(SMul(u, v), a) => pullV(SMul(u x a, v))
    case Cross(a, b) => if (nestedScalarsV(Cross(a, b))) {
      pullV(Cross(pullV(a), pullV(b)))
    } else {
      Cross(a, b)
    }

    case u: VExp => u
  }

  def pull(e: Exp): Exp = e match {
    case Add(u, v) => pull(u) + pull(v)
    case Mul(u, v) => pull(u) * pull(v)
    case Dot(SMul(u, v), w) => pull(v * Dot(u, w))
    case Dot(w, SMul(u, v)) => pull(v * Dot(u, w))
    case Dot(u, v) => if (nestedScalars(Dot(u, v))) {
      pull(Dot(pullV(u), pullV(v)))
    } else {
      Dot(u, v)
    }
    case u: Exp => u
  }


  // Main Simplification Function
  def simplify(e: Exp): Exp = {
    var s = expansion(e)
    s = pull(s)
    s = vectorRules(s)
    s = simp(s)
    return s
  }

  def simplifyV(v: VExp): VExp = {
    var u = pullV(v)
    u = simpV(u)
    return u
  }

}
