using UnityEngine;

[RequireComponent(typeof(SpriteRenderer))]
public class SpriteAlphaBreath : MonoBehaviour
{
    [Header("Alpha Range")]
    [Range(0f, 1f)] public float minAlpha = 0.2f;
    [Range(0f, 1f)] public float maxAlpha = 1.0f;

    [Header("Speed")]
    [Tooltip("Breathing speed (cycles per second roughly).")]
    public float speed = 1.0f;

    [Header("Optional")]
    public bool useUnscaledTime = false; // if true, ignores Time.timeScale

    private SpriteRenderer sr;

    void Awake()
    {
        sr = GetComponent<SpriteRenderer>();
        // 防止参数填反
        if (maxAlpha < minAlpha)
        {
            float t = maxAlpha;
            maxAlpha = minAlpha;
            minAlpha = t;
        }
    }

    void Update()
    {
        float t = useUnscaledTime ? Time.unscaledTime : Time.time;
        // pingpong: 0->1->0->1... 循环往返
        float p = Mathf.PingPong(t * speed, 1f);

        float a = Mathf.Lerp(minAlpha, maxAlpha, p);

        Color c = sr.color;
        c.a = a;
        sr.color = c;
    }
}
