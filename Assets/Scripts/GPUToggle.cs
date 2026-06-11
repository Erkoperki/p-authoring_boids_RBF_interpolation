using UnityEngine;

public class GPUToggle : MonoBehaviour
{
    [SerializeField] private bool useGPU = true;

    public bool UseGPU()
    {
        return useGPU;
    }
}
