using UnityEngine;

[RequireComponent(typeof(SpriteRenderer))]
public class SimpleWall : MonoBehaviour
{
    [Header("基础物理属性")]
    [Tooltip("摩擦力：0=滑，1=粘")]
    [Range(0f, 1f)]
    public float friction = 0.5f;

    private SpriteRenderer _renderer;
    void Awake() => _renderer = GetComponent<SpriteRenderer>();

    public Bounds GetBounds()
    {
        if (_renderer == null) _renderer = GetComponent<SpriteRenderer>();
        return _renderer.bounds;
    }

    void OnDrawGizmos()
    {
        Gizmos.color = Color.yellow;
        if (GetComponent<SpriteRenderer>() != null)
        {
            Gizmos.DrawWireCube(GetComponent<SpriteRenderer>().bounds.center, GetComponent<SpriteRenderer>().bounds.size);
        }
    }
}