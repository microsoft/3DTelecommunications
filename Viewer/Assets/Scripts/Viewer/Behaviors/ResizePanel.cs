using UnityEngine;
using UnityEngine.UI;
using Assets.Scripts.Viewer.Common;
using Assets.Scripts.Common.Extensions;
using AspectMode = UnityEngine.UI.AspectRatioFitter.AspectMode;
using UnityEngine.EventSystems;
using System.Linq;

namespace Assets.Scripts.Viewer.Behaviors
{
    public class ResizePanel : MonoBehaviour, IPointerEnterHandler, IPointerExitHandler
    {
        // https://docs.unity3d.com/ScriptReference/Input.GetMouseButtonDown.html
        private const int LEFT_BUTTON = 0;

        [SerializeField]
        public RectTransform validRect = null;

        [SerializeField]
        private int resizeWellSize = 30;

        [SerializeField]
        private Sprite resizeEWIcon;

        [SerializeField]
        private Sprite resizeNSIcon;

        [SerializeField]
        private Sprite resizeNWSEIcon;

        [SerializeField]
        private Sprite resizeSWNEIcon;

        [SerializeField]
        private Sprite moveIcon;

        [SerializeField]
        private int minPanelWidth = 50;

        [SerializeField]
        private int minPanelHeight = 50;

        [SerializeField]
        private bool resize = true;

        [SerializeField]
        private bool move = true;

        private bool isPointerOver = false;
        private bool prevIsPointerDown = false;
        private bool isUserMoving = false;
        private bool mouseCaptured = false;
        private bool isUserResizing = false;
        private UserResizeOrigin resizeOrigin;
        private AspectRatioFitter ratioFitter;

        private Vector2 pointerDownScreenPos;
        private Vector2 lastUpdateScreenPos;
        private RectTransform rectTransform;
        private RectTransform parentRectTransform;
        private CursorManager.CursorInfo moveCursor;
        private CursorManager.CursorInfo restoreCursor;
        private CursorManager.CursorInfo resizeSWNECursor;
        private CursorManager.CursorInfo resizeNWSECursor;
        private CursorManager.CursorInfo resizeEWCursor;
        private CursorManager.CursorInfo resizeNSCursor;

        private CursorManager.CursorInfo appliedCursor = null;

        /// <summary>
        /// OnEnable lifecycle function
        /// </summary>
        private void OnEnable()
        {
            rectTransform = transform as RectTransform;
            parentRectTransform = transform.parent as RectTransform;
            ratioFitter = GetComponent<AspectRatioFitter>();

            if (resizeEWIcon != null)
            {
                resizeEWCursor = new CursorManager.CursorInfo(
                    Utils.MakeTexture2DFromSprite(resizeEWIcon),
                    Utils.GetHotspotFromSprite(resizeEWIcon)
                );
            }

            if (resizeNSIcon != null)
            {
                resizeNSCursor = new CursorManager.CursorInfo(
                    Utils.MakeTexture2DFromSprite(resizeNSIcon),
                    Utils.GetHotspotFromSprite(resizeNSIcon)
                );
            }

            if (resizeNWSEIcon != null)
            {
                resizeNWSECursor = new CursorManager.CursorInfo(
                    Utils.MakeTexture2DFromSprite(resizeNWSEIcon),
                    Utils.GetHotspotFromSprite(resizeNWSEIcon)
                );
            }

            if (resizeSWNEIcon != null)
            {
                resizeSWNECursor = new CursorManager.CursorInfo(
                    Utils.MakeTexture2DFromSprite(resizeSWNEIcon),
                    Utils.GetHotspotFromSprite(resizeSWNEIcon)
                );
            }

            if (moveIcon != null)
            {
                moveCursor = new CursorManager.CursorInfo(
                    Utils.MakeTexture2DFromSprite(moveIcon),
                    Utils.GetHotspotFromSprite(moveIcon)
                );
            } 
            else
            {
                moveCursor = new CursorManager.CursorInfo(null);
            }
        }

        /// <summary>
        /// <inheritDoc />
        /// </summary>
        private void OnDisable()
        {
            // Remove any cursor we have applied
            CursorManager.RemoveCursor(appliedCursor);
        }

        /// <summary>
        /// Handles the Update loop
        /// </summary>
        private void Update()
        {
            bool isUserHoldingButton = Input.GetMouseButton(LEFT_BUTTON);

            // The user just started pressing the button
            if (Input.GetMouseButtonDown(LEFT_BUTTON))
            {
                // The user can move/resize if they started dragging on some child of ours
                mouseCaptured = Utils.GetGameObjectsUnderPoint(Input.mousePosition)
                    .Any(n => n.transform.IsChildOf(transform));
            }

            // As soon as the user releases the button, we're no longer capturing the mouse
            if (!isUserHoldingButton)
            {
                mouseCaptured = false;
            }

            bool isPointerDown = mouseCaptured && isUserHoldingButton;

            HandleResize(isPointerDown, prevIsPointerDown, isPointerOver);
            HandleMove(isPointerDown, prevIsPointerDown, isPointerOver);

            ClampBounds();

            UpdateCursor(isPointerOver);

            prevIsPointerDown = isPointerDown;
        }
        
        /// <summary>
        /// Handles the user moving the panel
        /// </summary>
        private void HandleMove(bool isPointerDown, bool prevIsPointerDown, bool isPointerOver)
        {
            if (move)
            {
                if (isPointerDown != prevIsPointerDown)
                {
                    pointerDownScreenPos = Input.mousePosition;
                    lastUpdateScreenPos = pointerDownScreenPos;
                    isUserMoving = isPointerOver && isPointerDown && !IsUserOverResizeWell();
                }

                if (isUserMoving)
                {
                    ApplyPanelTransform(Vector2.zero, (Vector2)Input.mousePosition - lastUpdateScreenPos);

                    lastUpdateScreenPos = Input.mousePosition;
                }
            }
        }

        /// <summary>
        /// Handles the user reszing the panel
        /// </summary>
        private void HandleResize(bool isPointerDown, bool prevIsPointerDown, bool isPointerOver)
        {
            if (resize)
            {
                if (isPointerDown != prevIsPointerDown)
                {
                    pointerDownScreenPos = Input.mousePosition;
                    lastUpdateScreenPos = pointerDownScreenPos;
                    if (isPointerOver)
                    {
                        isUserResizing = isPointerDown && IsUserOverResizeWell();
                        resizeOrigin = GetResizeOrigin();
                    }
                    else
                    {
                        isUserResizing = false;
                        resizeOrigin = UserResizeOrigin.None;
                    }
                }

                if (isUserResizing)
                {
                    Vector2 size, translate;

                    // Compute how the panel should translate & resize based on how the user is resizing
                    ComputePanelResizeTransform(out size, out translate);

                    ApplyPanelTransform(size, translate);

                    lastUpdateScreenPos = Input.mousePosition;
                }
            }
        }

        /// <summary>
        /// Clamps this element to the parent's bounds
        /// </summary>
        private void ClampBounds()
        {
            if (validRect != null)
            {
                rectTransform.Clamp(validRect);
            }
        }

        /// <summary>
        /// Updates the cursor to match the current state of the system
        /// </summary>
        private void UpdateCursor(bool isPointerOver)
        {
            bool userPerformingOperation = isUserMoving || isUserResizing;

            CursorManager.CursorInfo newCursor = appliedCursor;

            // Only change if the user is not already performing an operation
            if (!userPerformingOperation)
            {
                if (isPointerOver)
                {
                    if (move)
                    {
                        newCursor = moveCursor;
                    }

                    if (resize && IsUserOverResizeWell())
                    {
                        UserResizeOrigin origin = GetResizeOrigin();
                        if (origin == UserResizeOrigin.N || origin == UserResizeOrigin.S)
                        {
                            newCursor = resizeNSCursor;
                        }
                        else if (origin == UserResizeOrigin.E || origin == UserResizeOrigin.W)
                        {
                            newCursor = resizeEWCursor;
                        }
                        else if (origin == UserResizeOrigin.NE || origin == UserResizeOrigin.SW)
                        {
                            newCursor = resizeSWNECursor;
                        }
                        else
                        {
                            newCursor = resizeNWSECursor;
                        }
                    }
                } 
                else
                {
                    newCursor = null;
                }
            }

            if (appliedCursor != newCursor)
            {
                CursorManager.RemoveCursor(appliedCursor);
                appliedCursor = newCursor;
                CursorManager.AddCursor(newCursor);
            }
        }

        /// <summary>
        /// Returns the origin of the resize
        /// </summary>
        private UserResizeOrigin GetResizeOrigin()
        {
            UserResizeOrigin origin = UserResizeOrigin.None;
            Vector2 local;

            // The output of this
            if (RectTransformUtility.ScreenPointToLocalPointInRectangle(rectTransform, Input.mousePosition, null, out local))
            {
                Vector2 offset = new Vector2(local.x + (rectTransform.pivot.x * rectTransform.rect.width), rectTransform.rect.height - (local.y + (rectTransform.pivot.y * rectTransform.rect.height)));

                // Top
                if (offset.y <= resizeWellSize)
                {
                    origin = origin | UserResizeOrigin.N;
                }

                // Bottom
                else if (offset.y >= (rectTransform.rect.height - resizeWellSize))
                {
                    origin = origin | UserResizeOrigin.S;
                }

                // Left
                if (offset.x <= resizeWellSize)
                {
                    origin = origin | UserResizeOrigin.W;
                }

                // Right
                else if (offset.x >= (rectTransform.rect.width - resizeWellSize))
                {
                    origin = origin | UserResizeOrigin.E;
                }
            }

            return origin;
        }

        /// <summary>
        /// Returns true if the user is over the resize well at all
        /// </summary>
        /// <returns></returns>
        private bool IsUserOverResizeWell()
        {
            return resize && GetResizeOrigin() != UserResizeOrigin.None;
        }

        /// <summary>
        /// Applies the given transfomations to the panel
        /// </summary>
        /// <param name="size">The adjustment to the size</param>
        /// <param name="translate">The translation to the panel</param>
        private void ApplyPanelTransform(Vector2 size, Vector2 translate)
        {
            // Translate the panel
            rectTransform.position += new Vector3(translate.x, translate.y, rectTransform.position.z);

            // Should we resize at all
            if (Mathf.Abs(size.x) >= 0 || Mathf.Abs(size.y) >= 0)
            {
                // Make sure they don't resize into oblivion
                if ((rectTransform.rect.width + size.x) >= minPanelWidth &&
                    (rectTransform.rect.height - size.y) >= minPanelHeight)
                {
                    AspectMode originalMode = AspectMode.None;

                    // The aspectratiofitter will prevent resizing in some directions based on the mode
                    if (ratioFitter != null)
                    {
                        originalMode = ratioFitter.aspectMode;

                        // Are we resizing in the Y direction
                        if (originalMode == AspectMode.WidthControlsHeight && Mathf.Abs(size.y) > 0)
                        {
                            ratioFitter.aspectMode = AspectMode.HeightControlsWidth;
                        }
                        else if (originalMode == AspectMode.HeightControlsWidth && Mathf.Abs(size.x) > 0)
                        {
                            ratioFitter.aspectMode = AspectMode.WidthControlsHeight;
                        }
                    }

                    rectTransform.sizeDelta += new Vector2(size.x, -size.y);

                    // Restore original mode
                    if (ratioFitter != null)
                    {
                        ratioFitter.aspectMode = originalMode;
                    }
                }
            }
        }

        /// <summary>
        /// Computes the transformation to apply to the panel based on resizing
        /// </summary>
        /// <param name="size">The amount to resize the panel by</param>
        /// <param name="translate">The amount to translate the panel by</param>
        private void ComputePanelResizeTransform(out Vector2 size, out Vector2 translate)
        {
            Vector2 currentLocal;
            Vector2 lastLocal;
            RectTransformUtility.ScreenPointToLocalPointInRectangle(rectTransform, Input.mousePosition, null, out currentLocal);
            RectTransformUtility.ScreenPointToLocalPointInRectangle(rectTransform, lastUpdateScreenPos, null, out lastLocal);

            Vector2 pointerDelta = currentLocal - lastLocal;
            size = Vector2.zero;
            translate = Vector2.zero;
            if (resizeOrigin.HasFlag(UserResizeOrigin.E))
            {
                size.x = pointerDelta.x;
            }
            else if (resizeOrigin.HasFlag(UserResizeOrigin.W))
            {
                translate.x = pointerDelta.x;
                size.x = -pointerDelta.x;
            }

            if (resizeOrigin.HasFlag(UserResizeOrigin.S))
            {
                size.y = pointerDelta.y;
            }
            else if (resizeOrigin.HasFlag(UserResizeOrigin.N))
            {
                translate.y = pointerDelta.y;
                size.y = -pointerDelta.y;
            }
        }

        /// <summary>
        /// <inheritdoc />
        /// </summary>
        public void OnPointerEnter(PointerEventData eventData)
        {
            isPointerOver = true;
        }

        /// <summary>
        /// <inheritdoc />
        /// </summary>
        public void OnPointerExit(PointerEventData eventData)
        {
            isPointerOver = false;
        }

        /// <summary>
        /// A set of flags indicating where the user started resizing from
        /// </summary>
        private enum UserResizeOrigin
        {
            None = 0,
            N = 1,
            E = 2,
            S = 4,
            W = 8,
            NE = N | E,
            NW = N | W,
            SE = S | E,
            SW = S | W,
        }
    }
}