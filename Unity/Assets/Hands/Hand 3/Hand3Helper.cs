using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Hand3Helper : MonoBehaviour
{
    public DHHandHelper handModelReference;

    // The following functions are used to transfer DH parameters to the Hand3
    // model. The script makes assumptions about the construction of the Hand3,
    // as well as the DH parameters (embodied in the DHHandHelper.Model class).
    // The script works by associating multiple DH parameters with particular
    // transforms. Note that due to the way the DH parameterisation uses the a
    // parameter, sometimes the direction of rotation of the hinge axis is
    // inverted, amongst other small differences.
    // Therefore there fingers are initialised step by step rather than trying
    // to find a general algorithm to make the conversions.

    public void ImportFromDHParameters()
    {
        var model = handModelReference.GetModel();
        ImportFromDHModel(model.Index);
        ImportFromDHModel(model.Middle);
        ImportFromDHModel(model.Ring);
        ImportFromDHModel(model.Little);
        ImportFromDHModel(model.Thumb);
    }

    private void ImportFromDHModel(DHHandHelper.Finger model)
    {
        var finger = transform.Find(model.Name);

        finger.transform.localPosition = Vector3.zero;
        finger.transform.localRotation =
            Quaternion.AngleAxis(model.CMCrotation.th, Vector3.up) *
            Quaternion.AngleAxis(-model.CMCrotation.a, Vector3.forward);

        finger.GetComponent<Socket1D>().range = new Vector2(model.CMCflex.min_th, model.CMCflex.max_th);

        // Finger 1
        finger = finger.GetChild(0);

        finger.transform.localPosition = new Vector3(0, 0, model.CMCflex.r);
        finger.transform.localRotation =
            Quaternion.AngleAxis(-model.CMCflex.a, Vector3.forward) *
            Quaternion.AngleAxis(model.MCPabduc.th, Vector3.up) *
            Quaternion.AngleAxis(-model.MCPflex.th, Vector3.right);

        finger.GetComponent<Socket2D>().range_x = new Vector2(-model.MCPflex.max_th, -model.MCPflex.min_th) + Vector2.one * model.MCPflex.th;
        finger.GetComponent<Socket2D>().range_y = new Vector2(model.MCPabduc.min_th, model.MCPabduc.max_th) - Vector2.one * model.MCPabduc.th;

        // Finger 2
        finger = finger.GetChild(0);

        finger.localPosition = new Vector3(0, 0, model.MCPflex.r);
        finger.localRotation =
            Quaternion.AngleAxis(-model.PIP.th, Vector3.right);

        finger.GetComponent<Socket1D>().range = new Vector2(-model.PIP.max_th, -model.PIP.min_th) + Vector2.one * model.PIP.th;

        // Finger 3
        finger = finger.GetChild(0);

        finger.localPosition = new Vector3(0, 0, model.PIP.r);
        finger.localRotation =
            Quaternion.AngleAxis(-model.DIP.th, Vector3.right);

        finger.GetComponent<Socket1D>().range = new Vector2(-model.DIP.max_th, -model.DIP.min_th) + Vector2.one * model.DIP.th;

        // Tip
        finger = finger.GetChild(0);

        finger.localPosition = new Vector3(0, 0, model.DIP.r);
    }

    private void ImportFromDHModel(DHHandHelper.Thumb model)
    {
        var finger = transform.Find(model.Name);

        finger.transform.localPosition = Vector3.zero;
        finger.transform.localRotation =
            Quaternion.AngleAxis(model.CMCprojection.th, Vector3.up);

        // Thumb 1
        finger = finger.GetChild(0);
        finger.transform.localPosition = new Vector3(0, model.CMCprojection.d, model.CMCprojection.r);
        finger.transform.localRotation =
            Quaternion.AngleAxis(-model.CMCflex.th, Vector3.right);

        finger.GetComponent<Socket1D>().range = new Vector2(-model.CMCflex.max_th, -model.CMCflex.min_th) + Vector2.one * model.CMCflex.th;


        // Thumb 2
        finger = finger.GetChild(0);
        finger.transform.localPosition = new Vector3(0, 0, model.CMCflex.r);
        finger.transform.localRotation = 
            Quaternion.AngleAxis(model.CMCflex.a, Vector3.forward) *
            Quaternion.AngleAxis(-model.CMCabduc.th, Vector3.right);

        finger.GetComponent<Socket1D>().range = new Vector2(-model.CMCabduc.max_th, -model.CMCabduc.min_th) + Vector2.one * model.CMCabduc.th;

        // Thumb 3
        finger = finger.GetChild(0);
        finger.transform.localPosition = new Vector3(0, 0, model.CMCabduc.r);
        finger.transform.localRotation =
            Quaternion.AngleAxis(model.MCPflex.th, Vector3.right);

        finger.GetComponent<Socket2D>().range_x = new Vector2(model.MCPflex.min_th, model.MCPflex.max_th) + Vector2.one * model.MCPflex.th;
        finger.GetComponent<Socket2D>().range_y = new Vector2(model.MCPabduc.min_th, model.MCPabduc.max_th) - Vector2.one * model.MCPabduc.th;

        // Thumb 4
        finger = finger.GetChild(0);
        finger.transform.localPosition = new Vector3(0, 0, model.MCPflex.r);
        finger.transform.localRotation =
            Quaternion.AngleAxis(model.IP.th, Vector3.right);

        finger.GetComponent<Socket1D>().range = new Vector2(model.IP.min_th, model.IP.max_th) - Vector2.one * model.IP.th;

        // Thumb 5
        finger = finger.GetChild(0);
        finger.transform.localPosition = new Vector3(0, 0, model.IP.r);
        finger.transform.localRotation = Quaternion.identity;

    }
}
