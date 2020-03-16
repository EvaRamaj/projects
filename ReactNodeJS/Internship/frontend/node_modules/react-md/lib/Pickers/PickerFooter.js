'use strict';

Object.defineProperty(exports, "__esModule", {
  value: true
});

var _createClass = function () { function defineProperties(target, props) { for (var i = 0; i < props.length; i++) { var descriptor = props[i]; descriptor.enumerable = descriptor.enumerable || false; descriptor.configurable = true; if ("value" in descriptor) descriptor.writable = true; Object.defineProperty(target, descriptor.key, descriptor); } } return function (Constructor, protoProps, staticProps) { if (protoProps) defineProperties(Constructor.prototype, protoProps); if (staticProps) defineProperties(Constructor, staticProps); return Constructor; }; }();

var _react = require('react');

var _react2 = _interopRequireDefault(_react);

var _propTypes = require('prop-types');

var _propTypes2 = _interopRequireDefault(_propTypes);

var _classnames = require('classnames');

var _classnames2 = _interopRequireDefault(_classnames);

var _Button = require('../Buttons/Button');

var _Button2 = _interopRequireDefault(_Button);

function _interopRequireDefault(obj) { return obj && obj.__esModule ? obj : { default: obj }; }

function _classCallCheck(instance, Constructor) { if (!(instance instanceof Constructor)) { throw new TypeError("Cannot call a class as a function"); } }

function _possibleConstructorReturn(self, call) { if (!self) { throw new ReferenceError("this hasn't been initialised - super() hasn't been called"); } return call && (typeof call === "object" || typeof call === "function") ? call : self; }

function _inherits(subClass, superClass) { if (typeof superClass !== "function" && superClass !== null) { throw new TypeError("Super expression must either be null or a function, not " + typeof superClass); } subClass.prototype = Object.create(superClass && superClass.prototype, { constructor: { value: subClass, enumerable: false, writable: true, configurable: true } }); if (superClass) Object.setPrototypeOf ? Object.setPrototypeOf(subClass, superClass) : subClass.__proto__ = superClass; }

/**
 * The `PickerFooter` component is a dialog footer for the `Picker` components.
 * It renders an ok and cancel button.
 */
var PickerFooter = function (_PureComponent) {
  _inherits(PickerFooter, _PureComponent);

  function PickerFooter() {
    _classCallCheck(this, PickerFooter);

    return _possibleConstructorReturn(this, (PickerFooter.__proto__ || Object.getPrototypeOf(PickerFooter)).apply(this, arguments));
  }

  _createClass(PickerFooter, [{
    key: 'render',
    value: function render() {
      var _props = this.props,
          className = _props.className,
          okLabel = _props.okLabel,
          okPrimary = _props.okPrimary,
          onOkClick = _props.onOkClick,
          cancelLabel = _props.cancelLabel,
          cancelPrimary = _props.cancelPrimary,
          onCancelClick = _props.onCancelClick;


      return _react2.default.createElement(
        'footer',
        { className: (0, _classnames2.default)('md-dialog-footer', className) },
        _react2.default.createElement(_Button2.default, {
          flat: true,
          primary: cancelPrimary,
          secondary: !cancelPrimary,
          label: cancelLabel,
          onClick: onCancelClick
        }),
        _react2.default.createElement(_Button2.default, {
          flat: true,
          primary: okPrimary,
          secondary: !okPrimary,
          label: okLabel,
          onClick: onOkClick
        })
      );
    }
  }]);

  return PickerFooter;
}(_react.PureComponent);

PickerFooter.propTypes = {
  className: _propTypes2.default.string,
  okLabel: _propTypes2.default.node.isRequired,
  okPrimary: _propTypes2.default.bool.isRequired,
  onOkClick: _propTypes2.default.func.isRequired,
  cancelLabel: _propTypes2.default.node.isRequired,
  cancelPrimary: _propTypes2.default.bool.isRequired,
  onCancelClick: _propTypes2.default.func.isRequired
};
exports.default = PickerFooter;
//# sourceMappingURL=PickerFooter.js.map