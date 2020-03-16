"use strict";

import React from 'react';
import Styled from 'styled-components';


class PlainFooter extends React.Component {

    constructor(props) {
        super(props);
    }

    render() {
        return (
            <div className="Footer_custom">
                <div className="logo">
                    <span>LeazIt</span>
                </div>
                <div className="footer-details">
                    <span> Contact Us </span>
                    <span> Privacy </span>
                    <span> Terms </span>
                    <span><a href="#" className="fa fa-facebook"></a></span>
                    <span><a href="#" className="fa fa-twitter"></a></span>
                </div>
            </div>
        );
    }
}

export const Footer = Styled(PlainFooter)`

`;

