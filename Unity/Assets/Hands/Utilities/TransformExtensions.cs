using UnityEngine;
using System.Collections;

namespace UCL.CASMS
{
    public static class TransformExtensions
    {
        public static Transform FindDeepPartial(this Transform parent, string name)
        {
            if (parent.name.Contains(name))
            {
                return parent;
            }

            foreach (Transform child in parent)
            {
                var result = child.FindDeepPartial(name);
                if (result != null)
                {
                    return result;
                }
            }

            return null;
        }
        public static Transform FindDeep(this Transform parent, string name)
        {
            if (parent.name == name)
            {
                return parent;
            }

            foreach (Transform child in parent)
            {
                var result = child.FindDeep(name);
                if (result != null)
                {
                    return result;
                }
            }

            return null;
        }

        public static Transform ChainEnd(this Transform start)
        {
            foreach(Transform child in start)
            {
                return ChainEnd(child);
            }

            return start;
        }
             
    }
}