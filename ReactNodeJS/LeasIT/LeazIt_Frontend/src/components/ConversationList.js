"use strict";

import React from 'react';
import { DataTable, TableHeader, TableBody, TableRow, TableColumn, Button } from 'react-md';

import ConversationListRow  from './ConversationListRow';
import Page from './Page'
import { SimpleLink } from './SimpleLink';
import { withRouter } from 'react-router-dom'

class ConversationList extends React.Component{
    constructor(props) {
        super(props);
    }
    render(){
        return(
            <Page>
                <a href="http://localhost:8000/#/new_conversation/" class="button" style={{color:"#009900"}}>
                    +Add new Conversation
                </a>
                <div className="row" style={{backgroundColor:"#009900", border:"0.1px solid", height:"70px"}}>
                    <div className="col-2 justify-content-md-center conv_details" >
                        <span className="conv_more">Recipient</span>
                    </div>
                    <div className="col-8 justify-content-md-center conv_details">
                        <span className="conv_more">Message</span>
                    </div>
                    <div className="col-2 justify-content-md-center conv_details">
                        <span className="conv_more">View More</span>
                    </div>
                </div>
                <div className="row">
                    <div className="col bg-success">
                        {this.props.data.map((conversation, i) => <ConversationListRow key={i} conversation={conversation}/>)}
                    </div>
                </div>
            </Page>
        );
    }
}

export default withRouter(ConversationList)
