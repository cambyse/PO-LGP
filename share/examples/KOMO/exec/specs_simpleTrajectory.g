Include = '../../../data/keywords.g'
Include = '../easy/test.ors'

KOMO{
  T = 100
  duration = 5
}

(MinSumOfSqr qItself){ order=2 time=[0 1] }
(EqualZero posDiff endeff target){ vec1=[.15 0 0] time=[1 1] scale=100 }
(MinSumOfSqr pos endeff){ order=1 time=[1 1] scale=100 }
#(EqualZero vecAlign endeff target){ vec1=[1 0 0] vec2=[1 0 0] time=[1 1] target=[1] }
(LowerEqualZero collisionIneq){ margin=0.05 scale=.1 }
