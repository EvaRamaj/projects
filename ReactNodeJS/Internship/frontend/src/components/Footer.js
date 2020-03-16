"use strict";

import React from 'react';
import Styled from 'styled-components';
import {Link} from 'react-router-dom';
import youtube from '../assets/img/youtube.svg';
import share from '../assets/img/share.svg';
import twitter from '../assets/img/twitter.svg';
import facebook from '../assets/img/facebook.svg';

class PlainFooter extends React.Component {

    constructor(props) {
        super(props);
    }

    render() {
        return (
            <div className={this.props.className} style={{position: this.props.isSticky ? 'fixed' : undefined}}>
                <div className="col-md-12">
                    <div className="col-md-1"><Link to='/impressum'><p>Impressum</p></Link></div>
                    <div className="col-md-1"><Link to='/privacy'><p>Privacy Policy</p></Link></div>
                    <div className="col-md-1"><a href="mailto:support@xperienced.net"><p>Support</p></a></div>
                    <div className="col-md-6"/>
                    <div className="col-md-3">
                        <div className="col-md-5">
                            <p>Connect with us:</p>
                        </div>
                        <div className="col-md-7">
                            <div className="col-md-3"><img src={youtube}/></div>
                            <div className="col-md-3"><img src={share}/></div>
                            <div className="col-md-3"><img src={twitter}/></div>
                            <div className="col-md-3"><img src={facebook}/></div>
                        </div>
                    </div>
                </div>
            </div>
        );
    }
}

export const Footer = Styled(PlainFooter)`
    height: 50px;
    bottom: 0;
    left: 0;
    right: 0;
    background: white;
    > div {
        text-align: center;
        margin-top: 20px;
    }
`;
