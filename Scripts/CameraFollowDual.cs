using UnityEngine;

public class CameraFollowDual : MonoBehaviour
{
    public Transform player1;
    public Transform player2;

    [Header("跟随设置")]
    public float smoothTime = 0.2f;
    public Vector3 offset = new Vector3(0, 0, -10f);

    [Header("动态缩放")]
    public float minSize = 5f;
    public float maxSize = 12f;
    [Tooltip("间距达到多少时相机开始拉远")]
    public float zoomSensitivity = 0.8f;

    private Vector3 currentVelocity;
    private Camera cam;

    void Start() => cam = GetComponent<Camera>();

    void LateUpdate()
    {
        if (player1 == null || player2 == null) return;

        // 1. 追踪两人中点
        Vector3 midpoint = (player1.position + player2.position) / 2f;
        transform.position = Vector3.SmoothDamp(transform.position, midpoint + offset, ref currentVelocity, smoothTime);

        // 2. 根据距离动态缩放视野
        float distance = Vector3.Distance(player1.position, player2.position);
        float targetSize = Mathf.Clamp(distance * zoomSensitivity, minSize, maxSize);
        cam.orthographicSize = Mathf.Lerp(cam.orthographicSize, targetSize, Time.deltaTime * 3f);
    }
}