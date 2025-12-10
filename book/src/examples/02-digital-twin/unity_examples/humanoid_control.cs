using UnityEngine;

/// <summary>
/// Simple humanoid controller for Unity digital twin examples
/// This script allows basic movement and interaction with the humanoid model
/// </summary>
public class HumanoidController : MonoBehaviour
{
    [Header("Movement Settings")]
    public float moveSpeed = 5.0f;
    public float rotateSpeed = 100.0f;
    public float jumpForce = 7.0f;

    [Header("Ground Check")]
    public Transform groundCheck;
    public float groundCheckRadius = 0.1f;
    public LayerMask groundLayerMask;

    private Rigidbody rb;
    private bool isGrounded;
    private Animator animator;

    void Start()
    {
        rb = GetComponent<Rigidbody>();
        if (rb == null)
        {
            // Add Rigidbody if not present
            rb = gameObject.AddComponent<Rigidbody>();
            rb.mass = 1.0f;
            rb.drag = 1.0f;
            rb.angularDrag = 1.0f;
        }

        // Try to get animator component
        animator = GetComponent<Animator>();
    }

    void Update()
    {
        // Ground check
        isGrounded = Physics.CheckSphere(groundCheck.position, groundCheckRadius, groundLayerMask);

        // Handle input
        HandleMovement();
        HandleJump();
    }

    void HandleMovement()
    {
        // Get input
        float horizontal = Input.GetAxis("Horizontal");
        float vertical = Input.GetAxis("Vertical");

        // Calculate movement direction
        Vector3 movement = new Vector3(horizontal, 0, vertical);
        movement = Camera.main.transform.TransformDirection(movement);
        movement.y = 0; // Keep movement on horizontal plane

        // Apply movement
        transform.Translate(movement * moveSpeed * Time.deltaTime);

        // Rotate character to face movement direction
        if (movement != Vector3.zero)
        {
            Quaternion newRotation = Quaternion.LookRotation(movement);
            transform.rotation = Quaternion.Slerp(transform.rotation, newRotation, rotateSpeed * Time.deltaTime);
        }

        // Update animator parameters if available
        if (animator != null)
        {
            animator.SetFloat("Speed", movement.magnitude);
            animator.SetFloat("Horizontal", horizontal);
            animator.SetFloat("Vertical", vertical);
        }
    }

    void HandleJump()
    {
        if (Input.GetButtonDown("Jump") && isGrounded)
        {
            rb.AddForce(Vector3.up * jumpForce, ForceMode.Impulse);

            if (animator != null)
            {
                animator.SetTrigger("Jump");
            }
        }
    }

    void OnDrawGizmosSelected()
    {
        if (groundCheck != null)
        {
            Gizmos.color = isGrounded ? Color.green : Color.red;
            Gizmos.DrawWireSphere(groundCheck.position, groundCheckRadius);
        }
    }
}