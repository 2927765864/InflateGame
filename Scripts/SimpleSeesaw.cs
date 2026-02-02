using UnityEngine;

public class SimpleSeesaw : MonoBehaviour
{
    [Header("尺寸与物理")]
    public Vector2 size = new Vector2(8f, 0.5f); // 长而扁
    public float inertia = 50f;                 // 转动惯量：越大转动越迟钝
    [Range(0f, 1f)]
    public float angularDamping = 0.95f;        // 旋转阻尼
    public float friction = 0.2f;               // 表面摩擦力

    [Header("角度限制")]
    public float maxAngleDegree = 30f;          // 最大倾斜角度
    public float restoringForce = 2.0f;         // 自动回正的力量强度

    [HideInInspector] public float angle;       // 当前弧度
    private float prevAngle;
    private Vector2 pivotPosition;              // 固定中心点
    private SimpleWall[] allWalls;

    void Start()
    {
        pivotPosition = transform.position;
        angle = transform.rotation.eulerAngles.z * Mathf.Deg2Rad;
        prevAngle = angle;
        allWalls = FindObjectsOfType<SimpleWall>();
    }

    void FixedUpdate()
    {
        float dt = Time.fixedDeltaTime;

        // 1. 计算角速度（Verlet积分）
        float angularVelocity = (angle - prevAngle) * angularDamping;
        prevAngle = angle;
        angle += angularVelocity;

        // 2. 自动回正力矩（让它倾向于回到水平 0 度）
        float angleDiff = 0 - angle;
        angle += angleDiff * restoringForce * dt;

        // 3. 限制角度
        float maxRad = maxAngleDegree * Mathf.Deg2Rad;
        if (angle > maxRad) { angle = maxRad; prevAngle = angle; }
        if (angle < -maxRad) { angle = -maxRad; prevAngle = angle; }

        // 4. 环境碰撞（防止跷跷板转进墙里，虽然通常跷跷板下会有支架）
        ResolveWallCollisions();

        // 5. 应用变换
        transform.position = pivotPosition; // 强制锁定位置
        transform.rotation = Quaternion.Euler(0, 0, angle * Mathf.Rad2Deg);
    }

    // 同样由 SlimeSoftBody 调用
    public void AddImpulseAtPosition(Vector2 impulse, Vector2 worldPos)
    {
        // 跷跷板只接受力矩：Torque = r x F
        Vector2 r = worldPos - pivotPosition;
        float torque = r.x * impulse.y - r.y * impulse.x;
        angle += torque / inertia;
    }

    private void ResolveWallCollisions()
    {
        // 这里可以复用 SAT 逻辑，或者简单地让跷跷板只受角度限制不与墙体交互
        // 如果你的关卡设计中跷跷板下面有地面，建议保留角度限制即可，简单高效
    }

    public Vector2[] GetWorldCorners()
    {
        Vector2[] corners = new Vector2[4];
        Vector2 halfSize = size * 0.5f;
        corners[0] = RotatePoint(new Vector2(-halfSize.x, -halfSize.y));
        corners[1] = RotatePoint(new Vector2(halfSize.x, -halfSize.y));
        corners[2] = RotatePoint(new Vector2(halfSize.x, halfSize.y));
        corners[3] = RotatePoint(new Vector2(-halfSize.x, halfSize.y));
        return corners;
    }

    private Vector2 RotatePoint(Vector2 localPoint)
    {
        float cos = Mathf.Cos(angle);
        float sin = Mathf.Sin(angle);
        return new Vector2(
            pivotPosition.x + localPoint.x * cos - localPoint.y * sin,
            pivotPosition.y + localPoint.x * sin + localPoint.y * cos
        );
    }
}