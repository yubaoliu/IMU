using System;
using System.Collections.Generic;
using System.Linq;
using System.Runtime.InteropServices;
using System.Text;
using UnityEngine;

namespace Assets.scripts
{
  class DllWrapper
  {
    [DllImport("MadgwickDllLib")]
    public static extern bool IsMadgwickDllLibLoaded();

    [DllImport("MadgwickDllLib")]
    public static extern Quaternion MadgwickFilter(float dt, ref Vector3 gyro, ref Vector3 acc,ref Vector3 mag, Vector3 gyro_variance, Quaternion currentRotation);
  }
}
