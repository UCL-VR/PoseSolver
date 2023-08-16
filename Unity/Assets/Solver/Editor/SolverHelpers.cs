using System.Collections;
using System.Collections.Generic;
using System.Diagnostics;
using UnityEditor;
using UnityEngine;

public class SovlerHelpers
{
    [MenuItem("Solver/Open Depends...")]
    static void DoSomething()
    {
        var f = EditorUtility.OpenFilePanelWithFilters("Open Depends", "", new string[]{ "Executable", "exe" });
        if(f != null && f != "")
        {
            Process.Start(f);
        }
    }
}
