using Assets.Scripts.Viewer.Common;
using UnityEngine;
using UnityEngine.EventSystems;
using UnityEngine.UI;

namespace Assets.Scripts.Viewer.Behaviors
{
    public class PinchableScrollRect : ScrollRect, IPointerEnterHandler, IPointerExitHandler, IPointerClickHandler
    {
        //[SerializeField] 
        private float minZoom = 1f;

        //[SerializeField] 
        private float maxZoom = 10f;
        
        [SerializeField] 
        private float _zoomLerpSpeed = 10f;

        public bool zoomEnabled;
        public bool panEnabled;

        float _currentZoom = 1;
        bool _isPinching = false;
        float _startPinchDist;
        float _startPinchZoom;
        Vector2 _startPinchCenterPosition;
        Vector2 _startPinchScreenPosition;
        float _mouseWheelSensitivity = 1;
        bool blockPan = false;
        bool isPointerInside = false;
        bool isControlDown = false;

        private float zoomToolScaleFactor = 0.5f;

        public void ResetState()
        {
            _currentZoom = minZoom;
            content.localScale = Vector3.one * minZoom;
        }

        protected override void Awake()
        {
            Input.multiTouchEnabled = true;
        }

        private void Start()
        {
            if (SettingsManager.Instance)
            {
                zoomToolScaleFactor = SettingsManager.Instance.ZoomToolScaleFactor;
            }
        }

        protected override void OnDisable()
        {
            ResetState();
            base.OnDisable();
        }

        private void Update()
        {
            if (isPointerInside)
            {
                blockPan = !panEnabled;
                if (Input.touchCount == 2)
                {
                    if (!_isPinching)
                    {
                        _isPinching = true;
                        OnPinchStart();
                    }
                    OnPinch();
                }
                else
                {
                    _isPinching = false;
                    if (Input.touchCount == 0)
                    {
                        blockPan = false;
                    }
                }

                if (zoomEnabled)
                {
                    //pc input
                    float scrollWheelInput = Input.GetAxis("Mouse ScrollWheel");
                    if (Mathf.Abs(scrollWheelInput) > float.Epsilon)
                    {
                        _currentZoom *= 1 + scrollWheelInput * _mouseWheelSensitivity;
                        _currentZoom = Mathf.Clamp(_currentZoom, minZoom, maxZoom);
                        _startPinchScreenPosition = (Vector2)Input.mousePosition;
                        RectTransformUtility.ScreenPointToLocalPointInRectangle(content, _startPinchScreenPosition, null, out _startPinchCenterPosition);
                        Vector2 pivotPosition = new Vector3(content.pivot.x * content.rect.size.x, content.pivot.y * content.rect.size.y);
                        Vector2 posFromBottomLeft = pivotPosition + _startPinchCenterPosition;
                        SetPivot(content, new Vector2(posFromBottomLeft.x / content.rect.width, posFromBottomLeft.y / content.rect.height));
                    }
                    //pc input end

                    if (Mathf.Abs(content.localScale.x - _currentZoom) > 0.000001f)
                    {
                        content.localScale = Vector3.Lerp(content.localScale, Vector3.one * _currentZoom, _zoomLerpSpeed * Time.deltaTime);
                    }
                }

                var controlUpPressed = Utils.IsDeviceIndependentControlUp();
                var controlDownPressed = Utils.IsDeviceIndependentControlDown();
                if (controlDownPressed)
                {
                    isControlDown = true;
                } else if (controlUpPressed)
                {
                    isControlDown = false;
                }
            }
        }

        protected override void SetContentAnchoredPosition(Vector2 position)
        {
            if (_isPinching || blockPan) return;
            base.SetContentAnchoredPosition(position);
        }

        void OnPinchStart()
        {
            Vector2 pos1 = Input.touches[0].position;
            Vector2 pos2 = Input.touches[1].position;

            _startPinchDist = Distance(pos1, pos2) * content.localScale.x;
            _startPinchZoom = _currentZoom;
            _startPinchScreenPosition = (pos1 + pos2) / 2;
            RectTransformUtility.ScreenPointToLocalPointInRectangle(content, _startPinchScreenPosition, null, out _startPinchCenterPosition);

            Vector2 pivotPosition = new Vector3(content.pivot.x * content.rect.size.x, content.pivot.y * content.rect.size.y);
            Vector2 posFromBottomLeft = pivotPosition + _startPinchCenterPosition;

            SetPivot(content, new Vector2(posFromBottomLeft.x / content.rect.width, posFromBottomLeft.y / content.rect.height));
            blockPan = true;
        }

        void OnPinch()
        {
            float currentPinchDist = Distance(Input.touches[0].position, Input.touches[1].position) * content.localScale.x;
            _currentZoom = (currentPinchDist / _startPinchDist) * _startPinchZoom;
            _currentZoom = Mathf.Clamp(_currentZoom, minZoom, maxZoom);
        }

        float Distance(Vector2 pos1, Vector2 pos2)
        {
            RectTransformUtility.ScreenPointToLocalPointInRectangle(content, pos1, null, out pos1);
            RectTransformUtility.ScreenPointToLocalPointInRectangle(content, pos2, null, out pos2);
            return Vector2.Distance(pos1, pos2);
        }

        static void SetPivot(RectTransform rectTransform, Vector2 pivot)
        {
            if (rectTransform == null) return;

            Vector2 size = rectTransform.rect.size;
            Vector2 deltaPivot = rectTransform.pivot - pivot;
            Vector3 deltaPosition = new Vector3(deltaPivot.x * size.x, deltaPivot.y * size.y) * rectTransform.localScale.x;
            rectTransform.pivot = pivot;
            rectTransform.localPosition -= deltaPosition;
        }

        public void OnPointerEnter(PointerEventData eventData)
        {
            isPointerInside = true;
        }

        public void OnPointerExit(PointerEventData eventData)
        {
            isPointerInside = false;
        }

        public void OnPointerClick(PointerEventData eventData)
        {
            var tool = ViewerManager.Current().Store.GetState().ActiveTool.Value;
            if (tool == Models.ViewerTool.Zoom && zoomEnabled)
            {
                _currentZoom *= isControlDown ? 1 - zoomToolScaleFactor : 1 + zoomToolScaleFactor;
                _currentZoom = Mathf.Clamp(_currentZoom, minZoom, maxZoom);
                _startPinchScreenPosition = Input.mousePosition;
                RectTransformUtility.ScreenPointToLocalPointInRectangle(content, _startPinchScreenPosition, null, out _startPinchCenterPosition);
                Vector2 pivotPosition = new Vector3(content.pivot.x * content.rect.size.x, content.pivot.y * content.rect.size.y);
                Vector2 posFromBottomLeft = pivotPosition + _startPinchCenterPosition;
                SetPivot(content, new Vector2(posFromBottomLeft.x / content.rect.width, posFromBottomLeft.y / content.rect.height));
            }
        }
    }
}
