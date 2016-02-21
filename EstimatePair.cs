using System;
using System.Collections.Generic;
using System.Collections;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace ImprovedNonAtmosphericLandings
{
    class EstimatePair<T> : IEnumerable
    {
        public T lower;
        public T upper;

        public EstimatePair()
        {

        }

        public EstimatePair(T lower, T upper)
        {
            this.lower = lower;
            this.upper = upper;
        }

        public void Switch()
        {
            var temp = lower;
            lower = upper;
            upper = temp;
        }

        public T GetByIndex(int index)
        {
            switch (index)
            {
                case 0: return lower;
                case 1: return upper;
                default: throw new ArgumentOutOfRangeException("Only index of 0 or 1 is valid for this method.");
            }
        }

        public void SetByIndex(int index, T value)
        {
            switch (index)
            {
                case 0: lower = value; break;
                case 1: upper = value; break;
                default: throw new ArgumentOutOfRangeException("Only index of 0 or 1 is valid for this method.");
            }
        }

        public IEnumerator GetEnumerator()
        {
            yield return lower;
            yield return upper;
        }

        IEnumerator IEnumerable.GetEnumerator()
        {
            return GetEnumerator();
        }

    }
}
