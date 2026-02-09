using UnityEngine;

public class SimpleBox : MonoBehaviour
{
    [Header("物理参数")]
    public float mass = 10f;
    [Range(0f, 1f)]
    public float friction = 0.2f;      // 摩擦力
    [Range(0f, 1f)]
    public float groundFriction = 0.1f; // 地面摩擦
    public float linearDamping = 0.95f;  // 平移阻尼
    public float angularDamping = 0.94f; // 旋转阻尼

    public Vector2 size = new Vector2(1.5f, 1.5f);
    public Vector2 gravity = new Vector2(0, -35.0f);

    [Header("归平参数")]
    public float uprightSpeed = 2.0f; // 落地自动回正的速度

    [HideInInspector] public Vector2 position;
    [HideInInspector] public float angle; // 弧度

    private Vector2 prevPos;
    private float prevAngle;
    private float inertia;
    private SimpleWall[] allWalls;

    void Start()
    {
        position = transform.position;
        prevPos = position;
        angle = transform.rotation.eulerAngles.z * Mathf.Deg2Rad;
        prevAngle = angle;
        allWalls = FindObjectsOfType<SimpleWall>();

        // 计算转动惯量
        inertia = mass * (size.x * size.x + size.y * size.y) / 12f;
    }

    void FixedUpdate()
    {
        float dt = Time.fixedDeltaTime;

        // 1. Verlet 积分：计算平移和旋转速度
        Vector2 velocity = (position - prevPos) * linearDamping;
        float angularVelocity = (angle - prevAngle) * angularDamping;

        prevPos = position;
        prevAngle = angle;

        position += velocity + gravity * dt * dt;
        angle += angularVelocity;

        // 2. 自动归平（最短路径回正）
        if (Mathf.Abs(angularVelocity) < 0.2f)
        {
            float currentDeg = angle * Mathf.Rad2Deg;
            float targetDeg = Mathf.Round(currentDeg / 90f) * 90f;
            float degDiff = Mathf.DeltaAngle(currentDeg, targetDeg);
            float correction = (degDiff * Mathf.Deg2Rad) * uprightSpeed * dt;
            angle += correction;
            prevAngle += correction; // 抵消修正产生的虚假角速度
        }

        // 3. 处理环境碰撞（墙壁）
        ResolveEnvironmentCollisions();

        // 4. 处理动态碰撞（箱子与箱子）
        ResolveBoxCollisions();

        // 5. 同步视觉
        transform.position = position;
        transform.rotation = Quaternion.Euler(0, 0, angle * Mathf.Rad2Deg);
    }

    // 处理箱子与箱子之间的碰撞
    private void ResolveBoxCollisions()
    {
        SimpleBox[] allBoxes = FindObjectsOfType<SimpleBox>();
        foreach (var other in allBoxes)
        {
            if (other == this) continue;

            Vector2[] myCorners = GetWorldCorners();
            Vector2[] otherCorners = other.GetWorldCorners();

            // SAT检测轴：我的两个轴 + 对方的两个轴
            Vector2[] axes = new Vector2[4];
            axes[0] = new Vector2(Mathf.Cos(angle), Mathf.Sin(angle));
            axes[1] = new Vector2(-Mathf.Sin(angle), Mathf.Cos(angle));
            axes[2] = new Vector2(Mathf.Cos(other.angle), Mathf.Sin(other.angle));
            axes[3] = new Vector2(-Mathf.Sin(other.angle), Mathf.Cos(other.angle));

            float minOverlap = float.MaxValue;
            Vector2 smallestAxis = Vector2.zero;

            bool isIntersecting = true;
            foreach (Vector2 axis in axes)
            {
                if (!CheckProjectionOverlap(axis, myCorners, otherCorners, out float overlap))
                {
                    isIntersecting = false;
                    break;
                }
                if (overlap < minOverlap)
                {
                    minOverlap = overlap;
                    smallestAxis = axis;
                }
            }

            if (isIntersecting)
            {
                // 确保挤出方向是从 other 指向 this
                Vector2 dir = position - other.position;
                if (Vector2.Dot(smallestAxis, dir) < 0) smallestAxis = -smallestAxis;

                Vector2 separation = smallestAxis * minOverlap;

                // 简单的质量权重分配：两个箱子各退一半
                float totalMass = mass + other.mass;
                float myRatio = other.mass / totalMass;
                float otherRatio = mass / totalMass;

                position += separation * myRatio;
                other.position -= separation * otherRatio;

                // 速度交换（简单的碰撞能耗）
                prevPos = Vector2.Lerp(prevPos, position, 0.1f);
                other.prevPos = Vector2.Lerp(other.prevPos, other.position, 0.1f);
            }
        }
    }

    private void ResolveEnvironmentCollisions()
    {
        foreach (var wall in allWalls)
        {
            Bounds wB = wall.GetBounds();
            Vector2[] corners = GetWorldCorners();

            // 环境碰撞使用 AABB 简化轴
            Vector2[] axes = { Vector2.right, Vector2.up,
                               new Vector2(Mathf.Cos(angle), Mathf.Sin(angle)),
                               new Vector2(-Mathf.Sin(angle), Mathf.Cos(angle)) };

            float minOverlap = float.MaxValue;
            Vector2 smallestAxis = Vector2.zero;

            bool isIntersecting = true;
            foreach (var axis in axes)
            {
                if (!CheckWallOverlap(axis, corners, wB, out float overlap))
                {
                    isIntersecting = false;
                    break;
                }
                if (overlap < minOverlap) { minOverlap = overlap; smallestAxis = axis; }
            }

            if (isIntersecting)
            {
                Vector2 dir = position - (Vector2)wB.center;
                if (Vector2.Dot(smallestAxis, dir) < 0) smallestAxis = -smallestAxis;

                position += smallestAxis * minOverlap;

                // 落地归平力矩增强
                Vector2 contactPoint = corners[0];
                float maxD = float.MinValue;
                foreach (var c in corners)
                {
                    float d = Vector2.Dot((Vector2)wB.center - c, smallestAxis);
                    if (d > maxD) { maxD = d; contactPoint = c; }
                }
                Vector2 r = contactPoint - position;
                float torque = (r.x * smallestAxis.y * minOverlap - r.y * smallestAxis.x * minOverlap) * 2.0f;
                angle += torque;

                // 地面摩擦
                prevPos = Vector2.Lerp(prevPos, position, groundFriction);
                prevAngle = Mathf.Lerp(prevAngle, angle, 0.1f);
            }
        }
    }

    private bool CheckProjectionOverlap(Vector2 axis, Vector2[] cA, Vector2[] cB, out float overlap)
    {
        float minA = float.MaxValue, maxA = float.MinValue;
        foreach (var p in cA) { float dot = Vector2.Dot(p, axis); minA = Mathf.Min(minA, dot); maxA = Mathf.Max(maxA, dot); }
        float minB = float.MaxValue, maxB = float.MinValue;
        foreach (var p in cB) { float dot = Vector2.Dot(p, axis); minB = Mathf.Min(minB, dot); maxB = Mathf.Max(maxB, dot); }
        overlap = Mathf.Min(maxA, maxB) - Mathf.Max(minA, minB);
        return maxA > minB && maxB > minA;
    }

    private bool CheckWallOverlap(Vector2 axis, Vector2[] cA, Bounds wB, out float overlap)
    {
        float minA = float.MaxValue, maxA = float.MinValue;
        foreach (var p in cA) { float dot = Vector2.Dot(p, axis); minA = Mathf.Min(minA, dot); maxA = Mathf.Max(maxA, dot); }
        Vector2[] cB = { new Vector2(wB.min.x, wB.min.y), new Vector2(wB.max.x, wB.min.y), new Vector2(wB.max.x, wB.max.y), new Vector2(wB.min.x, wB.max.y) };
        float minB = float.MaxValue, maxB = float.MinValue;
        foreach (var p in cB) { float dot = Vector2.Dot(p, axis); minB = Mathf.Min(minB, dot); maxB = Mathf.Max(maxB, dot); }
        overlap = Mathf.Min(maxA, maxB) - Mathf.Max(minA, minB);
        return maxA > minB && maxB > minA;
    }

    public void AddImpulseAtPosition(Vector2 impulse, Vector2 worldPos)
    {
        position += impulse / mass;
        Vector2 r = worldPos - position;
        float torque = r.x * impulse.y - r.y * impulse.x;
        angle += torque / inertia;
    }

    public Vector2[] GetWorldCorners()
    {
        Vector2[] corners = new Vector2[4];
        Vector2 halfSize = size * 0.5f;
        float cos = Mathf.Cos(angle); float sin = Mathf.Sin(angle);
        Vector2[] locals = { new Vector2(-halfSize.x, -halfSize.y), new Vector2(halfSize.x, -halfSize.y), new Vector2(halfSize.x, halfSize.y), new Vector2(-halfSize.x, halfSize.y) };
        for (int i = 0; i < 4; i++) corners[i] = new Vector2(position.x + locals[i].x * cos - locals[i].y * sin, position.y + locals[i].x * sin + locals[i].y * cos);
        return corners;
    }

    void OnDrawGizmos()
    {
        Gizmos.color = Color.cyan;
        Vector2[] c = GetWorldCorners();
        for (int i = 0; i < 4; i++) Gizmos.DrawLine(c[i], c[(i + 1) % 4]);
    }
}